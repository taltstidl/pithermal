<p align="center">
  <img width="100rem" src="https://github.com/taltstidl/pithermal/blob/main/docs/_static/logo.svg" />
  <h1 align="center">pithermal</h1>
</p>

[![build_docs](https://github.com/taltstidl/pithermal/actions/workflows/build_docs.yml/badge.svg)](https://github.com/taltstidl/pithermal/actions/workflows/build_docs.yml)

> [!NOTE]  
> This library is currently in active development and only supports the Meridian Innovation Cougar MI0802 thermal image sensor at the time of writing. Builds are not versioned or released on PyPI. Expect breaking changes.

The `pithermal` Python library is meant to be an easy and straightforward way to communicate with supported thermal imaging cameras on a Raspberry Pi, in intent similar to what `picamera2` offers for normal visual cameras. It takes just a few lines of code to capture your first thermal image:

```python
from pithermal import MeridianPiThermal

camera = MeridianPiThermal()
camera.capture_file('thermal.png')
```
