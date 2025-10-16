import logging
import time

import cmap as cm
import numpy as np
from gpiozero import DigitalInputDevice, DigitalOutputDevice
from PIL import Image
from smbus import SMBus
from spidev import SpiDev

# Cyclic redundancy check function
import crcmod.predefined
crc16 = crcmod.predefined.mkCrcFun('crc-ccitt-false')

KELVIN_0 = -273.15  # in Celsius

RPI_GPIO_I2C_CHANNEL = 1  # ls /dev/*i2c* for available channels
MI48_I2C_ADDRESS = 0x40  # or 0x41

RPI_GPIO_SPI_BUS = 0  # ls /dev/*spi* for available buses and devices
RPI_GPIO_SPI_CE_MI48 = 0
MI48_SPI_MODE = 0b00
MI48_SPI_MAX_SPEED_HZ = 31200000
MI48_SPI_CS_DELAY = 0.0001


class MeridianPiThermal:
    """
    Class for use with Meridian Innovation thermal imaging sensors.

    .. _colormap catalog: https://pycmap.readthedocs.io/en/stable/catalog/
    """

    # Register addresses
    # see https://files.waveshare.com/wiki/Thermal-Camera-HAT/MI48x4--Datasheet--v4.0.5.pdf
    REG_FRAME_MODE = 0xB1
    REG_FW_VERSION_1 = 0xB2
    REG_FW_VERSION_2 = 0xB3
    REG_FRAME_RATE = 0xB4
    REG_POWER_DOWN_1 = 0xB5
    REG_STATUS = 0xB6
    REG_CLK_SPEED = 0xB7
    REG_MODULE_GAIN = 0xB9
    # REG_SENXOR_TYPE = 0xBA
    REG_MODULE_TYPE = 0xBB
    REG_SENSITIVITY_FACTOR = 0xC2
    REG_SELF_CALIBRATION = 0xC5
    REG_EMISSIVITY = 0xCA
    REG_OFFSET_CORR = 0xCB
    REG_SENXOR_ID = 0xE0  # 6 bytes
    REG_FILTER_CONTROL = 0xD0
    REG_FILTER_SETTING_1 = 0xD1  # 2 bytes
    REG_FILTER_SETTING_2 = 0xD3
    REG_USER_FLASH_CTRL = 0xD8

    # Status register values
    STATUS_READOUT_TOO_SLOW = 0x02
    STATUS_SENXOR_IF_ERROR = 0x04
    STATUS_CAPTURE_ERROR = 0x08
    STATUS_DATA_READY = 0x10
    STATUS_BOOTING = 0x20

    # Mode register values
    MODE_SINGLE = 0x01  #: Bit for single capture mode
    MODE_STREAM = 0x02  #: Bit for continuous capture mode
    MODE_READOUT = 0x1C  #: Bit for readout mode, currently only support full-frame
    MODE_NO_HEADER = 0x20  #: Bit for disabling frame headers

    def __init__(self):
        # Set up logger
        self.logger = logging.getLogger(type(self).__name__)
        # I2C interface
        self.i2c = SMBus(RPI_GPIO_I2C_CHANNEL)
        # SPI interface
        self.spi = SpiDev(RPI_GPIO_SPI_BUS, RPI_GPIO_SPI_CE_MI48)
        self.spi.mode = MI48_SPI_MODE
        self.spi.max_speed_hz = MI48_SPI_MAX_SPEED_HZ
        self.spi.bits_per_word = 8  # should be default, but just to be safe
        self.spi.lsbfirst = False  # should be default, but just to be safe
        # GPIO pins
        self.gpio_cs = DigitalOutputDevice("BCM7", active_high=False, initial_value=False)
        self.gpio_rst = DigitalOutputDevice("BCM23", active_high=False, initial_value=True)
        self.gpio_drdy = DigitalInputDevice("BCM24", pull_up=False)
        # Wait for boot-up sequence to complete
        self.reset()
        while self.status & MeridianPiThermal.STATUS_BOOTING:
            time.sleep(0.025)
        self.logger.debug('Finished booting')
        # Determine camera properties
        # TODO: Actually determine this based on reported module type
        self.image_shape = (80, 62)
        self.max_fps = 30

    def reset(self, assert_seconds=0.00005, deassert_seconds=0.05):
        self.gpio_rst.on()
        time.sleep(assert_seconds)
        self.gpio_rst.off()
        time.sleep(deassert_seconds)

    def _regread(self, addr):
        if isinstance(addr, str):
            addr = getattr(MeridianPiThermal, 'REG_' + addr)
        return self.i2c.read_byte_data(MI48_I2C_ADDRESS, addr)

    def _regwrite(self, addr, val):
        if isinstance(addr, str):
            addr = getattr(MeridianPiThermal, 'REG_' + addr)
        self.i2c.write_byte_data(MI48_I2C_ADDRESS, addr, val)

    @property
    def mode(self):
        """
        Current operating mode of the thermal image processor. Can be either single or continuous capturing with or
        without headers. If neither, no capturing takes place.
        """
        return self._regread('FRAME_MODE')

    @mode.setter
    def mode(self, value):
        self._regwrite('FRAME_MODE', value)

    @property
    def firmware_version(self):
        fwv = self._regread('FW_VERSION_1')
        fwb = self._regread('FW_VERSION_2')
        fwv_major = (fwv >> 4) & 0xF
        fwv_minor = fwv & 0xF
        fwv_build = fwb
        return '{}.{}.{}'.format(fwv_major, fwv_minor, fwv_build)

    @property
    def fps_divider(self):
        return self._regread('FRAME_RATE')

    @fps_divider.setter
    def fps_divider(self, value):
        self._regwrite('FRAME_RATE', value)

    @property
    def status(self):
        status = self._regread('STATUS')
        if status != 0:
            self.logger.warning('Non-zero STATUS: 0x{:02X}'.format(status))
            status_bits = []
            if status & MeridianPiThermal.STATUS_READOUT_TOO_SLOW:
                status_bits.append('Readout too slow')
            if status & MeridianPiThermal.STATUS_SENXOR_IF_ERROR:
                status_bits.append('SenXor interface error')
            if status & MeridianPiThermal.STATUS_CAPTURE_ERROR:
                status_bits.append('Capture error')
            if status & MeridianPiThermal.STATUS_DATA_READY:
                status_bits.append('Data ready')
            if status & MeridianPiThermal.STATUS_BOOTING:
                status_bits.append('Booting in progress')
            self.logger.debug('STATUS: ' + ','.join(status_bits))
        return status

    @property
    def serial_number(self):
        base_addr = MeridianPiThermal.REG_SENXOR_ID
        prod_year = self._regread(base_addr) + 2000
        prod_week = self._regread(base_addr + 1)
        mfg_loc = self._regread(base_addr + 2)
        serial_no = (self._regread(base_addr + 3) << 16) + (self._regread(base_addr + 4) << 8) \
                    + self._regread(base_addr + 5)
        return '{}.{}.{}.{}'.format(prod_year, prod_week, mfg_loc, serial_no)

    def _read_frame(self, with_header=True, crc_verify=True):
        self.gpio_drdy.wait_for_active()
        self.logger.debug('Retrieving frame')
        self.gpio_cs.on()
        time.sleep(MI48_SPI_CS_DELAY)

        cols, rows = self.image_shape
        xfer_size = cols * rows  # image
        if with_header:
            xfer_size += cols  # header
        xfer_bytes = 2 * xfer_size

        data = np.zeros(xfer_bytes, dtype='u1')
        xfer_step = 160  # size in bytes of a single transfer
        dummy_bytes = [0, ] * xfer_step
        for i in range(0, xfer_bytes, xfer_step):
            data[i:i + xfer_step] = np.array(self.spi.xfer2(dummy_bytes)).astype('u1')
        data = np.ndarray(buffer=data, shape=(xfer_size,), dtype='>u2')
        self.logger.debug('Received {} bytes'.format(data.shape[0]))

        time.sleep(MI48_SPI_CS_DELAY)
        self.gpio_cs.off()

        header, image = data[:-cols * rows], data[-cols * rows:]
        if with_header:
            frame_count = header[0]
            senxor_vdd = header[1] / 1.0e4
            senxor_temp = header[2] / 100. + KELVIN_0
            timestamp = (header[4] << 16) + header[3]
            max_temp = header[5] / 10. + KELVIN_0
            min_temp = header[6] / 10. + KELVIN_0
            crc = hex(header[7])
            if crc_verify:
                crc_ref = hex(crc16(data))
                if crc == crc_ref:
                    self.logger.warning('CRC failed')
                else:
                    self.logger.debug('CRC passed')

        image = image / 10. + KELVIN_0  # convert to temperature
        image = image.reshape((cols, rows), order='F').T.astype(np.float16)  # convert to proper image
        return image

    def capture_array(self):
        """ Capture a single thermal image as a raw temperature array. The temperature values are pre-converted
        to 16-bit floats in °C units.

        Returns
        -------
        np.ndarray
        """
        self.mode = MeridianPiThermal.MODE_SINGLE
        return self._read_frame()

    def capture_image(self, cmap: str = 'seaborn:rocket'):
        """ Capture a single thermal image as a PIL image.

        Parameters
        ----------
        cmap: str
            Color map for translating temperature values to image colors. See official `colormap catalog`_ for all
            available options.

        Returns
        -------
        PIL.Image
        """
        frame = self.capture_array()
        # Normalize for values in range [0, 1]
        frame = (frame - frame.min()) / (frame.max() - frame.min())
        # Apply color map
        colormap = cm.Colormap(cmap)
        image = 255. * colormap(frame)[..., :3]
        # Create and return Pillow image
        return Image.fromarray(image.astype(np.uint8), 'RGB')

    def capture_file(self, name='thermal.png', cmap='seaborn:rocket'):
        """ Save a single thermal image to disk.

        Parameters
        ----------
        name: str
            File path at which image should be saved. The image format is derived from the file
            extension and any format supported by PIL may be used.
        cmap: str
            Color map for translating temperature values to image colors. See official `colormap catalog`_ for all
            available options.
        """
        image = self.capture_image(cmap)
        image.save(name)

    def capture_files(self, name='thermal{:04d}.png', cmap='seaborn:rocket', num_files=1, delay=1):
        """ Save a sequence of thermal images to disk.

        Parameters
        ----------
        name: str
            File path at which image should be saved. The path should contain a format directive, otherwise the images
            will simply overwrite one another. The image format is derived from the file extension and any format
            supported by PIL may be used.
        cmap: str
            Color map for translating temperature values to image colors. See official `colormap catalog`_ for all
            available options.
        num_files: int | None
            Number of images that should be taken. If ``None`` will indefinitely collect images.
        delay: float
            Time between two images, in seconds. Note that the actual time may slightly deviate from the requested time
            due to internal rounding.
        """
        self.mode = MeridianPiThermal.MODE_STREAM
        self.fps_divider = int(round(delay * self.max_fps))
        colormap = cm.Colormap(cmap)
        # Continuously capture images until the desired number of files is reached
        captured_files = 0
        while num_files is None or captured_files < num_files:
            frame = self._read_frame()
            frame = (frame - frame.min()) / (frame.max() - frame.min())
            image = 255. * colormap(frame)[..., :3]
            Image.fromarray(image.astype(np.uint8), 'RGB').save(name.format(captured_files))
            captured_files += 1
        self.stop_capture()

    def stop_capture(self):
        # Only touch single and stream capture bits
        self.mode = self.mode & (~(MeridianPiThermal.MODE_SINGLE | MeridianPiThermal.MODE_STREAM) & 0xFF)

    def close(self):
        self.i2c.close()
        self.spi.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
