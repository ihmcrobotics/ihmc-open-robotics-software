## BlackMagic devices tools

Initially developed as an example integration between libavformat and the
bmd sdk, it ended up being a set of useful tools to use the BlackMagic Devices
decklink cards on Linux and MacOSX.

Thanks to TodoStreaming sponsoring its early development.

## Build instructions

In order to build it just clone/unpack this on your Sample directory from the
DeckLink SDK and then issue "make". If you have Libav and pkg-config installed
it will build fine. Make sure you are using a quite recent Libav otherwise it
will not build.

You can build it out of the Sample tree by issuing

```sh
make SDK_PATH=/path/to/the/bmd/include
```

### MacOSX Support

Pass SYS=Darwin to make, it should work, please DO complain to BlackMagic
Design for having non-matching apis cross operating systems.

### Windows Support

The tools do not build on Windows currently, supporting it would either
require a working widl support in mingw64 or access to the native tools.

Patch and/or sponsorship welcome.

## Usage

```sh
./bmdcapture -C 1 -m 2 -I 1 -F nut -f pipe:1 | avconv -y -i - <your options here>
```

-I switch from the default (HDMI) source to Analog (both audio and video)

-C select the capture device if more than one is present.

-F define the container format, I suggest using nut.

-f output file name, any libavformat compatible url is supported.

-m specific modeline, resolution+framerate

NOTE: make sure you are processing frames capture in real time or be
prepared to end up using all your memory quite quickly, HD raw data
fills up memory quickly.

```sh
avconv -vsync 1 -i <source> -c:v rawvideo -pix_fmt uyvy422 -c:a pcm_s16le -ar 48000 -f nut - | ./bmdplay -f pipe:0
```

## Support

The github [issue tracker](https://github.com/lu-zero/bmdtools/issues) can
be used to track bugs and feature requests.

### Contact

You can directly contact me either at

lu_zero@gentoo.org or luca.barbato@luminem.it

### Paid Support

Paid support is offered, contact info@luminem.it for details.
