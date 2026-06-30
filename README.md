# Deblock

Reduces the blockiness of the image using the deblocking filter from h264.

Ported from AviSynth plugin http://www.avisynth.nl/users/fizick/mvtools/deblock.html.


## Parameters

```py
deblock.Deblock(vnode clip[, int quant=25, int aoffset=0, int boffset=0, int[] planes=[0, 1, 2], int opt=0])
```

- clip: Clip to process. Any format with either 8-16 bit depth integer sample type or 32 bit depth float sample type is supported.

- quant: The higher the quant, the stronger the deblocking. Valid range is 0 ... 60.

- aoffset: A quant modifier for the blockiness detection threshold. Larger values mean stronger edge deblocking.

- boffset: Another quant modifier, used both for the blockiness detector and for the deblocking strength. Here too: the higher it is, the stronger the effect.

- planes: Specifies which planes will be processed. Any unprocessed planes will be simply copied.

- opt: Specifies which cpu optimizations to use.
  - 0 = auto detect
  - 1 = use c
  - 2 = use sse4.1

If `quant` + `aoffset` is less to 16, no filtering is performed at all. The same applies to `quant` + `boffset`.


## Installation

```
pip install -U vapoursynth-deblock
```
