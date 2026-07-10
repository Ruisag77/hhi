#include "EipLog.h"

#include <array>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <unordered_set>

namespace EipLog
{
namespace
{
namespace fs = std::filesystem;

enum class Role
{
  NONE,
  ENCODER,
  DECODER
};

struct ChangedSample
{
  bool     encoder { false };
  int      poc { 0 };
  int      x { 0 };
  int      y { 0 };
  int      width { 0 };
  int      height { 0 };
  unsigned cand { 0 };
  unsigned inferredMode { 0 };
  int      oldHor { 0 };
  int      oldVer { 0 };
  int      newHor { 0 };
  int      newVer { 0 };
};

struct LogState
{
  Role     role { Role::NONE };
  fs::path logFile;
  bool     initialized { false };
  bool     finished { false };

  std::atomic<uint64_t> finalEip { 0 };
  std::atomic<uint64_t> finalEipWithResidual { 0 };
  std::atomic<uint64_t> implicitMts { 0 };
  std::atomic<uint64_t> changedMts { 0 };

  std::mutex                      mutex;
  std::unordered_set<std::string> decoderImplicitBlocks;
  std::array<ChangedSample, 8>    samples;
  size_t                          numSamples { 0 };
};

LogState &state()
{
  static LogState logState;
  return logState;
}

std::string normalizePath(const std::string &path)
{
  if (path.empty())
  {
    return {};
  }

  std::error_code ec;
  fs::path        absolutePath = fs::absolute(fs::path(path), ec);
  return (ec ? fs::path(path) : absolutePath).lexically_normal().string();
}

std::string sanitizeName(std::string name)
{
  for (char &c: name)
  {
    const unsigned char value = static_cast<unsigned char>(c);
    if (!std::isalnum(value) && c != '_' && c != '-' && c != '.')
    {
      c = '_';
    }
  }
  return name.empty() ? "unknown_sequence" : name;
}

std::string sequenceFromInput(const std::string &inputFile)
{
  fs::path path(inputFile);
  return sanitizeName(path.stem().string());
}

int qpFromName(const std::string &name)
{
  std::string lower = name;
  for (char &c: lower)
  {
    c = char(std::tolower(static_cast<unsigned char>(c)));
  }

  size_t pos = lower.rfind("qp");
  if (pos == std::string::npos)
  {
    return -1;
  }
  pos += 2;
  if (pos >= lower.size() || !std::isdigit(static_cast<unsigned char>(lower[pos])))
  {
    return -1;
  }

  int qp = 0;
  while (pos < lower.size() && std::isdigit(static_cast<unsigned char>(lower[pos])))
  {
    qp = qp * 10 + lower[pos] - '0';
    pos++;
  }
  return qp;
}

std::string sequenceFromBitstream(const std::string &bitstreamFile)
{
  std::string stem  = fs::path(bitstreamFile).stem().string();
  std::string lower = stem;
  for (char &c: lower)
  {
    c = char(std::tolower(static_cast<unsigned char>(c)));
  }

  const size_t qpPos = lower.rfind("qp");
  if (qpPos != std::string::npos)
  {
    size_t end = qpPos;
    while (end > 0 && (stem[end - 1] == '_' || stem[end - 1] == '-'))
    {
      end--;
    }
    stem.resize(end);
  }
  return sanitizeName(stem);
}

fs::path logRoot()
{
  std::error_code ec;
  fs::path        root = fs::current_path(ec) / "EIP_LOG";
  return ec ? fs::path("EIP_LOG") : root;
}

bool ensureParentDirectory(const fs::path &file)
{
  std::error_code ec;
  fs::create_directories(file.parent_path(), ec);
  if (ec)
  {
    std::fprintf(stderr, "EIP log: cannot create directory %s: %s\n", file.parent_path().string().c_str(),
                 ec.message().c_str());
  }
  return !ec;
}

fs::path findEncoderLog(const std::string &bitstreamFile)
{
  const fs::path root = logRoot();
  std::error_code ec;
  if (!fs::exists(root, ec))
  {
    return {};
  }

  const std::string wanted = normalizePath(bitstreamFile);
  for (fs::recursive_directory_iterator it(root, fs::directory_options::skip_permission_denied, ec), end; it != end;
       it.increment(ec))
  {
    if (ec)
    {
      ec.clear();
      continue;
    }
    if (!it->is_regular_file(ec) || it->path().extension() != ".log")
    {
      continue;
    }

    std::ifstream input(it->path());
    std::string   header;
    if (!std::getline(input, header))
    {
      continue;
    }
    const std::string marker = " bitstream=";
    const size_t      pos    = header.find(marker);
    if (pos != std::string::npos && header.substr(pos + marker.size()) == wanted)
    {
      return it->path();
    }
  }
  return {};
}

void appendLine(const fs::path &file, const std::string &line)
{
  if (file.empty() || !ensureParentDirectory(file))
  {
    return;
  }
  std::ofstream output(file, std::ios::out | std::ios::app);
  if (output)
  {
    output << line << '\n';
  }
}

void resetState(LogState &logState, Role role, const fs::path &logFile)
{
  logState.role        = role;
  logState.logFile     = logFile;
  logState.initialized = true;
  logState.finished    = false;
  logState.finalEip.store(0);
  logState.finalEipWithResidual.store(0);
  logState.implicitMts.store(0);
  logState.changedMts.store(0);
  logState.decoderImplicitBlocks.clear();
  logState.numSamples = 0;
}

void saveChangedSample(LogState &logState, const ChangedSample &sample)
{
  std::lock_guard<std::mutex> lock(logState.mutex);
  if (logState.numSamples < logState.samples.size())
  {
    logState.samples[logState.numSamples++] = sample;
  }
}

void finish(Role role)
{
  LogState &logState = state();
  std::lock_guard<std::mutex> lock(logState.mutex);
  if (!logState.initialized || logState.finished || logState.role != role)
  {
    return;
  }
  logState.finished = true;

  std::ostringstream summary;
  if (role == Role::ENCODER)
  {
    summary << "ENC selected_eip_calls=" << logState.finalEip.load() << " mts_calls=" << logState.implicitMts.load()
            << " changed_calls=" << logState.changedMts.load();
  }
  else
  {
    summary << "DEC final_eip=" << logState.finalEip.load()
            << " final_eip_with_residual=" << logState.finalEipWithResidual.load()
            << " mts_blocks=" << logState.implicitMts.load() << " changed_blocks=" << logState.changedMts.load();
  }
  appendLine(logState.logFile, summary.str());

  for (size_t i = 0; i < logState.numSamples; i++)
  {
    const ChangedSample &sample = logState.samples[i];
    std::ostringstream   line;
    line << "CHANGED " << (sample.encoder ? "ENC" : "DEC") << " poc=" << sample.poc << " pos=" << sample.x << ','
         << sample.y << " size=" << sample.width << 'x' << sample.height << " cand=" << sample.cand
         << " inferred=" << sample.inferredMode << " old=H" << sample.oldHor << "/V" << sample.oldVer << " new=H"
         << sample.newHor << "/V" << sample.newVer;
    appendLine(logState.logFile, line.str());
  }
}
}   // namespace

void initEncoder(const std::string &inputFile, const std::string &bitstreamFile, int qp)
{
  LogState &logState = state();
  std::lock_guard<std::mutex> lock(logState.mutex);
  if (logState.initialized)
  {
    return;
  }

  const std::string sequence = sequenceFromInput(inputFile);
  const fs::path    logFile  = logRoot() / sequence / ("QP" + std::to_string(qp) + ".log");
  resetState(logState, Role::ENCODER, logFile);
  if (!ensureParentDirectory(logFile))
  {
    return;
  }

  std::ofstream output(logFile, std::ios::out | std::ios::trunc);
  if (output)
  {
    output << "META sequence=" << sequence << " qp=" << qp << " bitstream=" << normalizePath(bitstreamFile) << '\n';
  }
}

void initDecoder(const std::string &bitstreamFile)
{
  LogState &logState = state();
  std::lock_guard<std::mutex> lock(logState.mutex);
  if (logState.initialized)
  {
    return;
  }

  fs::path logFile = findEncoderLog(bitstreamFile);
  if (logFile.empty())
  {
    const int         qp       = qpFromName(bitstreamFile);
    const std::string sequence = sequenceFromBitstream(bitstreamFile);
    logFile = logRoot() / sequence / (qp >= 0 ? "QP" + std::to_string(qp) + ".log" : "QPunknown.log");
    if (ensureParentDirectory(logFile) && !fs::exists(logFile))
    {
      std::ofstream output(logFile, std::ios::out | std::ios::trunc);
      if (output)
      {
        output << "META sequence=" << sequence << " qp=" << qp << " bitstream=" << normalizePath(bitstreamFile)
               << '\n';
      }
    }
  }
  resetState(logState, Role::DECODER, logFile);
}

void finishEncoder() { finish(Role::ENCODER); }

void finishDecoder() { finish(Role::DECODER); }

void recordEncoderSelectedEip()
{
  LogState &logState = state();
  if (logState.initialized && logState.role == Role::ENCODER)
  {
    logState.finalEip.fetch_add(1, std::memory_order_relaxed);
  }
}

void recordDecoderFinalEip(bool hasResidual)
{
  LogState &logState = state();
  if (logState.initialized && logState.role == Role::DECODER)
  {
    logState.finalEip.fetch_add(1, std::memory_order_relaxed);
    if (hasResidual)
    {
      logState.finalEipWithResidual.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

void recordImplicitMts(bool encoder, int poc, int x, int y, int width, int height, unsigned cand,
                       unsigned inferredMode, int oldHor, int oldVer, int newHor, int newVer)
{
  LogState &logState = state();
  const Role expectedRole = encoder ? Role::ENCODER : Role::DECODER;
  if (!logState.initialized || logState.role != expectedRole)
  {
    return;
  }

  const bool changed = oldHor != newHor || oldVer != newVer;
  if (!encoder)
  {
    std::ostringstream key;
    key << poc << ':' << x << ':' << y << ':' << width << ':' << height << ':' << cand;
    std::lock_guard<std::mutex> lock(logState.mutex);
    if (!logState.decoderImplicitBlocks.insert(key.str()).second)
    {
      return;
    }
  }

  logState.implicitMts.fetch_add(1, std::memory_order_relaxed);
  if (changed)
  {
    logState.changedMts.fetch_add(1, std::memory_order_relaxed);
    saveChangedSample(logState, ChangedSample { encoder, poc, x, y, width, height, cand, inferredMode, oldHor, oldVer,
                                               newHor, newVer });
  }
}
}   // namespace EipLog
