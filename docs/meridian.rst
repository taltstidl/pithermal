Meridian Innovation
===================

`Meridian Innovation <https://meridianinno.com/>`_ is a Singapore-headquartered company (with operations also in Hong Kong, USA, and UK) specializing in low-cost, mass-producible long-wave infrared (LWIR) thermal imaging sensors made using a CMOS compatible process named SenXor™. Their line of products includes the *Cougar MI0802* **80x62** pixel sensor and the newer *Panther MI1602* **160x120** pixel sensor.

.. autoclass:: pithermal.MeridianPiThermal

   .. automethod:: __init__

   .. rubric:: Registers

   The thermal imaging processor exposes status information and configuration options through I2C and various registers. Many register values are in turn available here as properties.

   .. autoattribute:: MODE_SINGLE
   .. autoattribute:: MODE_STREAM
   .. autoattribute:: MODE_READOUT
   .. autoattribute:: MODE_NO_HEADER
   .. autoproperty:: mode

   .. rubric:: Capturing images

   .. automethod:: capture_array
   .. automethod:: capture_image
   .. automethod:: capture_file
   .. automethod:: capture_files
