Meridian Innovation
===================

`Meridian Innovation <https://meridianinno.com/>`_ is a Singapore-headquartered company (with operations also in Hong Kong, USA, and UK) specializing in low-cost, mass-producible long-wave infrared (LWIR) thermal imaging sensors made using a CMOS compatible process named SenXor™. Their line of products includes the *Cougar MI0802* **80x62** pixel sensor and the newer *Panther MI1602* **160x120** pixel sensor.

.. autoclass:: pithermal.MeridianPiThermal

   .. automethod:: __init__

   .. rubric:: Registers

   The thermal image processor exposes status information and configuration options in registers accessible through I2C. Some registers' values are also available here as properties. You should not normally need to use those directly but it may aid in debugging any issues that may occur.

   .. autoattribute:: MODE_SINGLE
      :noindex:
   .. autoattribute:: MODE_STREAM
      :noindex:
   .. autoattribute:: MODE_NO_HEADER
      :noindex:
   .. autoproperty:: mode
   .. autoproperty:: firmware_version
   .. autoproperty:: fps_divider
   .. autoattribute:: STATUS_READOUT_TOO_SLOW
      :noindex:
   .. autoattribute:: STATUS_SENXOR_IF_ERROR
      :noindex:
   .. autoattribute:: STATUS_CAPTURE_ERROR
      :noindex:
   .. autoattribute:: STATUS_DATA_READY
      :noindex:
   .. autoattribute:: STATUS_BOOTING
      :noindex:
   .. autoproperty:: status
   .. autoattribute:: FILTER_TEMP_ENABLE
      :noindex:
   .. autoattribute:: FILTER_TEMP_INIT
      :noindex:
   .. autoattribute:: FILTER_ROLL_AVG_ENABLE
      :noindex:
   .. autoattribute:: FILTER_MEDIAN_K
      :noindex:
   .. autoattribute:: FILTER_MEDIAN_ENABLE
      :noindex:
   .. autoproperty:: filter

   .. rubric:: Calibrating

   .. warning:: The module comes factory-calibrated. If you change any of the calibration settings you may not be able to recover the original settings provided from the factory. Only proceed if you know what you are doing.

   .. autoproperty:: calibration_offset
   .. autoproperty:: calibration_factor
   .. autoproperty:: emissivity
   .. automethod:: self_calibrate
   .. automethod:: factory_calibrate

   .. rubric:: Configuring filters

   The thermal image processor supports some built-in filtering techniques, namely temporal, rolling average, and median filtering. Since these are software-based they are turned off by default.

   .. autoproperty:: is_temporal_filter_enabled
   .. automethod:: configure_temporal_filter
   .. autoproperty:: is_rolling_avg_filter_enabled
   .. automethod:: configure_rolling_avg_filter
   .. autoproperty:: is_median_filter_enabled
   .. automethod:: configure_median_filter

   .. rubric:: Capturing images

   .. automethod:: capture_array
   .. automethod:: capture_image
   .. automethod:: capture_file
   .. automethod:: capture_files
