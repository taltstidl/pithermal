Quickstart
==========

.. toctree::
   :caption: Tutorials
   :hidden:

   self

.. toctree::
   :caption: Devices
   :hidden:

   meridian

The ``pithermal`` Python library is meant to be an easy and straightforward way to communicate with supported thermal imaging cameras on a Raspberry Pi, in intent similar to what ``picamera2`` offers for normal visual cameras. It takes just a few lines of code to capture your first thermal image:

.. code-block:: python

   from pithermal import MeridianPiThermal

   camera = MeridianPiThermal()
   camera.capture_file('thermal.png')

Supported Devices
-----------------

Currently only the Meridian Innovation Cougar MI0802 sensor is supported, in particular the Waveshare Long-wave IR Thermal Imaging Camera HAT (B) for Raspberry Pi. There are plans to support the higher-resolution Panther MI1602 sensor and the FLIR Lepton series but as of writing I did not purchase the necessary hardware yet.