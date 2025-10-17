Meridian Innovation
===================

`Meridian Innovation <https://meridianinno.com/>`_ is a Singapore-headquartered company (with operations also in Hong Kong, USA, and UK) specializing in low-cost, mass-producible long-wave infrared (LWIR) thermal imaging sensors made using a CMOS compatible process named SenXor™. Their line of products includes the *Cougar MI0802* **80x62** pixel sensor and the newer *Panther MI1602* **160x120** pixel sensor.

.. autoclass:: pithermal.MeridianPiThermal

   .. automethod:: __init__

   .. rubric:: Registers

   The thermal imaging processor exposes status information and configuration options in registers accessible through I2C. Some registers' values are also available here as properties. You should not normally need to use those directly but it may aid in debugging any issues that may occur.

   .. autoattribute:: MODE_SINGLE
   .. autoattribute:: MODE_STREAM
   .. autoattribute:: MODE_NO_HEADER
   .. autoproperty:: mode
   .. autoproperty:: firmware_version
   .. autoproperty:: fps_divider
   .. autoattribute:: STATUS_READOUT_TOO_SLOW
   .. autoattribute:: STATUS_SENXOR_IF_ERROR
   .. autoattribute:: STATUS_CAPTURE_ERROR
   .. autoattribute:: STATUS_DATA_READY
   .. autoattribute:: STATUS_BOOTING
   .. autoproperty:: status

   .. rubric:: Capturing images

   .. automethod:: capture_array
   .. automethod:: capture_image
   .. automethod:: capture_file
   .. automethod:: capture_files
