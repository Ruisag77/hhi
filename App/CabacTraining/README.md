# CABAC training

## 1- Dump CABAC bins
Run the decoder with the config parameter ActivateCABACDumping set equal to 1.

For each configurations (RA, AI, LDB, LDP) create a directory and decode all bitstream. 
Example:
``sh
mkdir -p AI;
cd AI;
for bs in /path/to/AI/*.bin; do
 DecoderAppStatic -b $bs --ActivateCABACDumping=1
``
It will create N files (N is the total number of contexts) for each decoded bitstream.
All bistreams can be decoded in parallel (the output filename is deduced from the the bitstream name).

## 2- Train a context

Build the CabacTrain application.
Send all the file related to a context to the trainer.

Example:
``sh
CTX=821;
cat AI/*.cabac_${CTX} RA/*.cabac_${CTX} LDB/*.cabac_${CTX} LDP/*.cabac_${CTX} | cabacTrain --output=context_${CTX}.txt
``

The resulting parameters are in the file:
``
821 A 26 41 35 7 7 8 4 4 18 99 115 99 115 99 115
``
821 is the context number, ``A`` can be ignored. Following numbers are the cabac parameters.

All contexts can be trained in parallel.



