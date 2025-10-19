<p align="center">
  <img width="100rem" src="https://github.com/taltstidl/pithermal/blob/main/docs/_static/logo.svg" />
  <h1 align="center">pithermal</h1>
</p>

[![build_docs](https://github.com/taltstidl/pithermal/actions/workflows/build_docs.yml/badge.svg)](https://github.com/taltstidl/pithermal/actions/workflows/build_docs.yml)

> [!NOTE]  
> This library is currently in active development and only supports the Meridian Innovation Cougar MI0802 thermal image sensor at the time of writing. Expect breaking changes in alpha releases.

The `pithermal` Python library is meant to be an easy and straightforward way to communicate with supported thermal imaging cameras on a Raspberry Pi, in intent similar to what `picamera2` offers for normal visual cameras. It takes just a few lines of code to capture your first thermal image:

```python
from pithermal import MeridianPiThermal

camera = MeridianPiThermal()
camera.capture_file('thermal.png')
```

## Installation

This library targets Raspberry Pi devices and has only a limited set of dependencies to make installation as straightforward as possible. It has been tested on Bullseye (Python 3.9) but should also work with Bookworm (Python 3.11) and Trixie (Python 3.13). In most cases it should be sufficient to install through PyPI:

```bash
pip3 install pithermal
```

## Supported Devices

* Meridian Innovation Cougar MI0802 thermal camera tested on *Waveshare Long-wave IR Thermal Imaging Camera HAT (B)* for Raspberry Pi (available for purchase [here](https://www.waveshare.com/thermal-45-camera-hat-b.htm?sku=28566))

There are plans to support the higher-resolution Panther MI1602, the FLIR Lepton series, and Seek Thermal devices but as of writing I did not purchase the necessary hardware yet.
