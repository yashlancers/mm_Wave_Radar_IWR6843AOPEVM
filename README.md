# mm-Wave-Radar-IWR6843AOPEVM

Download the folder mm-Wave-Radar-IWR6843AOPEVM
config_file.cfg is out of box demo configuration file i created using TI visualiser. You can use any configuration file of the radar as per your application flashed pn mm wave radar. Some demo applications are there on TI resources https://dev.ti.com/tirex/explore/node?node=AHJY4qNCowO17wH-P2ICKQ Configuration files for many demo projects are already included in the folder downloaded from this link. INcase making your own application , you will require custom config file. 
radar_serial_capture.py sends the configuration of config_file.cfg to sensor and starts the sensor. You can either use the config file in this folder or rename yours and add to this folder after download.
radar_data_dump.py will show you the raw data stream in terminal and will dump it in .txt file in the data folder with date time stamp for each session
parser_mmw_demo.py is the python script provided by TI for data parsing. The parket frame and other details can be understood from this to write your own data parsing file depending upon the intended application
