import logging
import time

from pithermal import MeridianPiThermal


def main():
    logging.basicConfig(
        format='%(asctime)s %(levelname)-8s %(name)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        level=logging.DEBUG)
    camera = MeridianPiThermal()
    camera.capture_file('thermal.png')
    camera.close()


if __name__ == '__main__':
    main()
