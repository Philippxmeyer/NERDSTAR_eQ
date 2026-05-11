FOCAL_LENGTH_MM  = 326.0
APERTURE_MM      = 50.0
PIXEL_SIZE_UM    = 2.0
SENSOR_WIDTH_PX  = 3840
SENSOR_HEIGHT_PX = 2160
SERIAL_PORT      = "/dev/ttyUSB0"
SERIAL_BAUD      = 9600
STORAGE_PATH     = "/mnt/storage"
ASTAP_PATH       = "/usr/bin/astap"
CATALOG_PATH     = "/home/pi/smartscope/data/catalog.xml"

# Optional shutdown park target.
# Leave both values as None to disable park motion on shutdown.
# If both are set, SmartScope will send a regular LX200 GoTo (:Sr/:Sd/:MS)
# before shutting down the host.
PARK_RA_HOURS    = None
PARK_DEC_DEG     = None
