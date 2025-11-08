# Readme

## Bitfield header generation tool

This directory contains the tool and sources to generate the C header files containing BGT60
register definition.

'generate_c_header.py' is a python script that reads an XML register description file and extracts
register and bitfield information from it. The extracted information is formatted in C style syntax
and written to the console.

The XML register description files were downloaded from the Chip Design Workspace through a Unix
account. Different files describe different register set versions:

- IFX_BGT60TRxxC_ID0000.xml
  This is the description of the first shared reticle version of BGT60TR24C. At that time there was
  no CHIP_ID register, but the register number nowadays used for CHIP_ID was unused and set to 0, so
  this version is considered as digital chip ID 0.

- IFX_BGT60TRxxC_ID0003.xml
  This is the description of the final BGT60TR13C with digital chip ID 3.

- IFX_BGT60TRxxC_ID0005.xml
  This is the description of the planned BGT60ATR24C with digital chip ID 5.

- IFX_BGT60TRxxD_ID0006.xml
  This is the description of the first version of BGT60TR13D with digital chip ID 6.

- IFX_BGT60TRxxD_ID0007.xml
  This is the description of the first version of BGT60TR11D with digital chip ID 7.

- IFX_BGT60TRxxE_ID0008.xml
  This is the description of the first version of BGT60TR13E with digital chip ID 8.

- IFX_BGT60TRxxE_ID0009.xml
  This is the description of the first version of BGT60ATR24E with digital chip ID 9.

- IFX_BGT120TR24E_ID000A.xml
  This is the description of the first version of BGT120UTR24 with digital chip ID 10.


## Avian devices and digital ids

| Device Name                | Digitial Id |
|:---------------------------|------------:|
| BGT60TR24B                 | 0           |
| BGT60TR24B (MONO)          | 1           |
| BGT60TR12C                 | 2           |
| BGT60TR12C (MIRRORED)      | 2           |
| BGT60TR13C                 | 3           |
| BGT60ATR13C                | 4           |
| BGT60ATR24C                | 5           |
| BGT24ATR24C                | 5           |
| BGT60TR13D                 | 6           |
| BGT60TR13D_GF              | 6           |
| BGT60TR13D_GF (V2)         | 6           |
| BGT60TR11D                 | 7           |
| BGT60TR11D (V2)            | 7           |
| BGT60TR11DS                | 7           |
| BGT60TR11DS_1V8            | 8           |
| BGT60TR12E                 | 8           |
| BGT120TR13E                | 8           |
| BGT60ATR24E                | 9           |
| BGT24LTR13E                | 9           |
| BGT120UTR24E               | 10          |
| BGT120TR24E1               | 10          |
| BGT120TR24E2 (V2)          | 10          |

**Watch out**: For digital ids 1, 2, 4 no hardware was produced!