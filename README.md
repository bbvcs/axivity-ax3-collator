# Axivity AX-Series Data CSV Collation Tool

A simple C++ command line tool designed to read Light, Temperature and Accelerometry (Gyro on AX6 and Gyro/Mag on AX9 should work also) data from Axivity AX-Series Data Logger .CWA file and merge into a single .CSV file.

Tested to work using a 14-day AX3 (100Hz Accelerometry) .CWA file (should work for other setups too), example output:

| timestamp     | accel_x   | accel_y  | accel_z   | light   | temperature |
|---------------|-----------|----------|-----------|---------|-------------|
| 1655288789460 | 0.34375   | -0.6875  | -0.765625 | 11810.4 | 31.4453     |
| 1655288789470 | -0.75     | 0.515625 | -0.21875  |         |             |
| 1655288789480 | -0.8125   | 0.53125  | -0.234375 |         |             |
| 1655288789490 | -0.84375  | 0.546875 | -0.21875  |         |             |
| 1655288789500 | -0.890625 | 0.546875 | -0.21875  |         |             |
| 1655288789510 | -0.96875  | 0.578125 | -0.1875   |         |             |
| 1655288789520 | -0.984375 | 0.640625 | -0.171875 |         |             |
| 1655288789530 | -0.984375 | 0.65625  | -0.140625 |
....

In this instance, there were 120 accelerometer samples per packet (packets are not tied uniquely to seconds). Light and Temperature are only recorded once per packet, so would not appear again until the 120th row.

Timestamps are *milliseconds since unix epoch*. Typically, unix epoch timestamps are in *seconds* since unix epoch. To convert to the standard format, remove the last 3 digits from a timestamp (1655288789460 becomes 1655288789). 

This project is a fork of James Christie and Andrew Wood's [Axivity AX3 Light and Temperature Extractor](https://github.com/jlc-christie/axivity-ax3-tool) tool (GPL v3), with code also taken from the ["cwa-convert"](https://github.com/digitalinteraction/openmovement/blob/master/Software/AX3/cwa-convert/c/main.c) code (BSD 2) of Daniel Jackson's (OpenLab, Newcastle University) official OMGUI Program. To adhere with licensing, source code taken from the OMGUI project has been noted as such.

This program is not well-tested due to lack of variety of data, so use at your own risk - there may be inaccuracies in results. Please inspect the code before usage. I would recommend the official OMGUI software for best results.

## Install
1. Clone this Repo:
`git clone "https://github.com/bbvcs/axivity-ax3-collator.git"`
2. Compile using a C++ 11 compatible compiler:
`g++ -std=c++11 -Wall main.cpp -o ax-collator`

## Usage
`/ax-collator -i [input file].cwa -o [output file].csv`


Billy C. Smith, Newcastle University, UK
Created while working as a summer student for the CNNP Lab, Newcastle University.
(No association with the Open Movement Team / Open Lab @ Newcastle University)



