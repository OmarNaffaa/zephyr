# HomeMonitor Project

Firmware for ESP32 DevKitC to collect data from ESP32 and upload data to ThingSpeak

Follow steps on https://docs.zephyrproject.org/latest/develop/getting_started/index.html to install packages or activate venv that is already set up properly

Compile for boards with "west build -p always -b esp32_devkitc_wrover/esp32/procpu homemonitor"
* [1/5/2024] Note that this could change if esp32's board hierarchy is restructured, and if a new command is needed the overaly file will also need to be renamed