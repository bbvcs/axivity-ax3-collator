#include <iostream>
#include <ctime>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>
#include <cmath>
#include <iterator> 

#include <bitset>

using namespace std;

// <CWA_CONVERT> <START>
// Access the packed i-th 4-byte value in the buffer in an endian-agnostic way:
#define PACKED_VALUE(buffer, i) ((uint32_t)((uint8_t *)buffer)[30 + i * 4] | ((uint32_t)((uint8_t *)buffer)[31 + i * 4] << 8) | ((uint32_t)((uint8_t *)buffer)[32 + i * 4] << 16) | ((uint32_t)((uint8_t *)buffer)[33 + i * 4] << 24))
// Split the x/y/z/ values out, using the supplied exponent:
#define UNPACK_X(value) ((short)( (short)((unsigned short)0xffc0 & (unsigned short)(value <<  6)) >> (6 - ((unsigned char)(value >> 30))) ))
#define UNPACK_Y(value) ((short)( (short)((unsigned short)0xffc0 & (unsigned short)(value >>  4)) >> (6 - ((unsigned char)(value >> 30))) ))
#define UNPACK_Z(value) ((short)( (short)((unsigned short)0xffc0 & (unsigned short)(value >> 14)) >> (6 - ((unsigned char)(value >> 30))) ))
// <CWA_CONVERT> <END>

struct cwa_timestamp {
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t hours;
  uint16_t mins;
  uint16_t seconds;
};

// <CWA_CONVERT> <START>

struct OM_READER_HEADER_PACKET {
	uint16_t packetHeader;                      ///< @ 0  +2   ASCII "MD", little-endian (0x444D)
	uint16_t packetLength;                      ///< @ 2  +2   Packet length (1020 bytes, with header (4) = 1024 bytes total)
	uint8_t  hardwareType;                      ///< @ 4  +1 * Hardware type (0x00/0xff/0x17 = AX3, 0x64 = AX6)
	uint16_t deviceId;                          ///< @ 5  +2   Device identifier (lower 16-bits)
	uint32_t sessionId;                         ///< @ 7  +4   Unique session identifier
	uint16_t upperDeviceId;                     ///< @11  +2 * Upper word of device id (if 0xffff is read, treat as 0x0000)
	cwa_timestamp loggingStartTime;           ///< @13  +4   Start time for delayed logging
	cwa_timestamp loggingEndTime;             ///< @17  +4   Stop time for delayed logging
	uint32_t loggingCapacity;                   ///< @21  +4   (Deprecated: preset maximum number of samples to collect, 0 = unlimited)
	uint8_t  reserved1[1];                      ///< @25  +1   (1 byte reserved)
	uint8_t  flashLed;                          ///< @26  +1   Flash LED during recording
	uint8_t  reserved2[8];                      ///< @27  +8   (8 bytes reserved)
	uint8_t  sensorConfig;                      ///< @35  +1 * Fixed rate sensor configuration, 0x00 or 0xff means accel only, otherwise bottom nibble is gyro range (8000/2^n dps): 2=2000, 3=1000, 4=500, 5=250, 6=125, top nibble non-zero is magnetometer enabled.
	uint8_t  samplingRate;                      ///< @36  +1   Sampling rate code, frequency (3200/(1<<(15-(rate & 0x0f)))) Hz, range (+/-g) (16 >> (rate >> 6)).
	cwa_timestamp lastChangeTime;             ///< @37  +4   Last change metadata time
	uint8_t  firmwareRevision;                  ///< @41  +1   Firmware revision number
	int16_t  timeZone;                          ///< @42  +2   (Unused: originally reserved for a "Time Zone offset from UTC in minutes", 0xffff = -1 = unknown)
	uint8_t  reserved3[20];                     ///< @44  +20  (20 bytes reserved)
	uint8_t  annotation[448];      ///< @64  +448 Scratch buffer / meta-data (448 ASCII characters, ignore trailing 0x20/0x00/0xff bytes, url-encoded UTF-8 name-value pairs)
	uint8_t  reserved[512];                     ///< @512 +512 Reserved for device-specific meta-data (512 bytes, ASCII characters, ignore trailing 0x20/0x00/0xff bytes, url-encoded UTF-8 name-value pairs, leading '&' if present?)
};

struct OM_READER_DATA_PACKET {
    uint16_t packetHeader;                      ///< @ 0  +2   ASCII "AX", little-endian (0x5841)
    uint16_t packetLength;                      ///< @ 2  +2   Packet length (508 bytes, with header (4) = 512 bytes total)
    uint16_t deviceFractional;                  ///< @ 4  +2   Top bit set: 15-bit fraction of a second for the time stamp, the timestampOffset was already adjusted to minimize this assuming ideal sample rate; Top bit clear: 15-bit device identifier, 0 = unknown;
    uint32_t sessionId;                         ///< @ 6  +4   Unique session identifier, 0 = unknown
    uint32_t sequenceId;                        ///< @10  +4   Sequence counter (0-indexed), each packet has a new number (reset if restarted)
    uint32_t timestamp;                			///< @14  +4   Last reported RTC value, 0 = unknown
    uint16_t lightScale;                        ///< @18  +2   AAAGGGLLLLLLLLLL Bottom 10 bits is last recorded light sensor value in raw units, 0 = none; top three bits are unpacked accel scale (1/2^(8+n) g); next three bits are gyro scale	(8000/2^n dps)
    uint16_t temperature;                       ///< @20  +2   Last recorded temperature sensor value in raw units (bottom-10 bits), 0 = none; (top 6-bits reserved)
    uint8_t  events;                            ///< @22  +1   Event flags since last packet, b0 = resume logging, b1 = reserved for single-tap event, b2 = reserved for double-tap event, b3 = reserved, b4 = reserved for diagnostic hardware buffer, b5 = reserved for diagnostic software buffer, b6 = reserved for diagnostic internal flag, b7 = reserved)
    uint8_t  battery;                           ///< @23  +1   Last recorded battery level in scaled/cropped raw units (double and add 512 for 10-bit ADC value), 0 = unknown
    uint8_t  sampleRate;                        ///< @24  +1   Sample rate code, frequency (3200/(1<<(15-(rate & 0x0f)))) Hz, range (+/-g) (16 >> (rate >> 6)).
    uint8_t  numAxesBPS;                        ///< @25  +1   0x32 (top nibble: number of axes, 3=Axyz, 6=Gxyz/Axyz, 9=Gxyz/Axyz/Mxyz; bottom nibble: packing format - 2 = 3x 16-bit signed, 0 = 3x 10-bit signed + 2-bit exponent)
    int16_t  timestampOffset;                   ///< @26  +2   Relative sample index from the start of the buffer where the whole-second timestamp is valid
    uint16_t sampleCount;                       ///< @28  +2   Number of sensor samples (if this sector is full -- Axyz: 80 or 120 samples, Gxyz/Axyz: 40 samples)
    uint8_t  rawSampleData[480];                ///< @30  +480 Raw sample data.  Each sample is either 3x/6x/9x 16-bit signed values (x, y, z) or one 32-bit packed value (The bits in bytes [3][2][1][0]: eezzzzzz zzzzyyyy yyyyyyxx xxxxxxxx, e = binary exponent, lsb on right)
    uint16_t checksum;                          ///< @510 +2   Checksum of packet (16-bit word-wise sum of the whole packet should be zero)
};

// <CWA_CONVERT> <END>

void process_header(char header_buffer[], OM_READER_HEADER_PACKET &header) {
/*
  memcpy(&header.packetHeader,     header_buffer,       2);
  memcpy(&header.packetLength,     header_buffer + 2,   2);
  memcpy(&header.reserved1,        header_buffer + 4,   1);
  memcpy(&header.deviceId,         header_buffer + 5,   2);
  memcpy(&header.sessionId,        header_buffer + 7,   4);
  memcpy(&header.reserved2,        header_buffer + 11,  2);
  memcpy(&header.loggingStartTime, header_buffer + 13,  4);
  memcpy(&header.loggingEndTime,   header_buffer + 17,  4);
  memcpy(&header.loggingCapacity,  header_buffer + 21,  4);
  memcpy(&header.reserved3,        header_buffer + 25,  11);
  memcpy(&header.samplingRate,     header_buffer + 36,  1);
  memcpy(&header.lastChangeTime,   header_buffer + 37,  4);
  memcpy(&header.firmwareRevision, header_buffer + 41,  1);
  memcpy(&header.timeZone,         header_buffer + 42,  2);
  memcpy(&header.reserved4,        header_buffer + 44,  20);
  memcpy(&header.annotation,       header_buffer + 64,  448);
  memcpy(&header.reserved,         header_buffer + 512, 512);
  * */
}

void process_data_block(char data_buffer[], OM_READER_DATA_PACKET &data) {
  memcpy(&data.packetHeader,       data_buffer,       2);
  memcpy(&data.packetLength,       data_buffer + 2,   2);
  memcpy(&data.deviceFractional,   data_buffer + 4,   2);
  memcpy(&data.sessionId,          data_buffer + 6,   4);
  memcpy(&data.sequenceId,         data_buffer + 10,  4);
  memcpy(&data.timestamp,          data_buffer + 14,  4);
  memcpy(&data.lightScale,              data_buffer + 18,  2);
  memcpy(&data.temperature,        data_buffer + 20,  2);
  memcpy(&data.events,             data_buffer + 22,  1);
  memcpy(&data.battery,            data_buffer + 23,  1);
  memcpy(&data.sampleRate,         data_buffer + 24,  1);
  memcpy(&data.numAxesBPS,         data_buffer + 25,  1);
  memcpy(&data.timestampOffset,    data_buffer + 26,  2);
  memcpy(&data.sampleCount,        data_buffer + 28,  2);
  memcpy(&data.rawSampleData,      data_buffer + 30,  480);
  memcpy(&data.checksum,           data_buffer + 510, 2);
}

void process_cwa_timestamp(uint32_t raw, cwa_timestamp &timestamp) {
  timestamp.year  = ((raw >> 26) & 0x3f) + 2000;
  timestamp.month = ((raw >> 22) & 0x0f);
  timestamp.day   = ((raw >> 17) & 0x1f);
  timestamp.hours = ((raw >> 12) & 0x1f);
  timestamp.mins  = ((raw >> 6)  & 0x3f);
  timestamp.seconds  =   raw     & 0x3f;
}

void print_cwa_timestamp(cwa_timestamp timestamp){
	
	printf("%04d-%02d-%02d %02d:%02d:%02d\n", timestamp.year,
                                                         timestamp.month,
                                                         timestamp.day,
                                                         timestamp.hours,
                                                         timestamp.mins,
                                                         timestamp.seconds);
                                                         
 }

time_t ax3_timestamp_to_unix_epoch(cwa_timestamp &ax3_timestamp){
	                                                  
    
    // https://stackoverflow.com/questions/21679056/convert-date-to-unix-time-stamp-in-c
    
    time_t rawtime;
    time(&rawtime); // initialise with current time

	// create tm struct using current time
    struct tm* timeinfo;
    timeinfo = gmtime(&rawtime); // covert local time to UTC
    
    // replace fields in time struct to reflect the AX3 timestamp
    timeinfo->tm_year = ax3_timestamp.year - 1900;
    timeinfo->tm_mon = ax3_timestamp.month -1;
    timeinfo->tm_mday = ax3_timestamp.day;
    timeinfo->tm_hour = ax3_timestamp.hours;
    timeinfo->tm_min = ax3_timestamp.mins;
    timeinfo->tm_sec = ax3_timestamp.seconds;
    
    time_t unix_epoch = mktime(timeinfo); 
   
    return unix_epoch;
	
} 


string add_ms_to_unix_epoch(time_t unix_epoch, short ms) {
	
	// add ms to timestamp (ms is not part of unix epoch standard)
    // simply adding ms on the end gives ms since unix epoch I believe
    
    
    if (ms > 999){
		cout << "Warning: ms should only be 0-999" << endl;
    }
    
    char unix_epoch_ms[14];
    snprintf(unix_epoch_ms, 14, "%ld%03d", unix_epoch, ms);
    
    return unix_epoch_ms;
}


void collate_and_convert(ifstream &file, string out_filename) {//, vector<string> &timestamps, vector<cwa_timestamp> &cwa_timestamps) {

  bool first_block {true};
		
  int64_t recording_start = 0;
  int64_t recording_end = 0;
  
  
  vector<int64_t> ms_timestamps {};
  
  vector<double> data_light {};
  vector<double> data_temperature {}; 
  
  // Accelerometer Buffers (AX3, AX6, AX9)
  vector<double> data_x {};
  vector<double> data_y {};
  vector<double> data_z {};
  
  // Gyroscope Buffers (AX6, AX9)
  vector<double> data_gx {};
  vector<double> data_gy {};
  vector<double> data_gz {};
  
  // Magnetometer Buffers (AX9)
  vector<double> data_mx {};
  vector<double> data_my {};
  vector<double> data_mz {};
  
  //cout << sum16(data_buffer, sizeof(data_buffer)) << endl;
  
  // variables we need for writing, so declare in this scope:
  short num_axes {};
  short sample_count {};
	
  while (file.peek() != EOF) {
	  
    char data_buffer[512];
    file.read(data_buffer, 512);
    
    OM_READER_DATA_PACKET data_packet;
    process_data_block(data_buffer, data_packet);
    
    
    float sample_rate = 3200.0f / (1 << (15 - (data_packet.sampleRate & 0x0f)));
	sample_count = data_packet.sampleCount;
    
    /* WARNING:
     * Sample Rate likely not the same as Sample Count.
     * Sample Rate == samples per second (100Hz - 400Hz)
     * Sample Count == samples per packet (only ever 40, 80, 120)
     * Data blocks are not synced to time, i.e each packet != 1s of real time.
     * 
     * The recording starts at some point in time, then records continuously
     * at the given sample rate.
     * 
     * This may be confusing, as each packet/block has a "timestamp" field,
     * which increases by 1s every packet.
     * 
     * So we can use the "timestampOffset" field to work out when the packet
     * actually started, which may be slightly before/after this timestamp.
     * 
     * Furthermore, the number of axes influences how the samples should be read.
     * In an AX3, with non-packed samples, the first 3 16-bit samples per block
     * wil be accelerometer, and so will the 3 that follow it.
     * But, in an AX6, the first 3 will be gyro, and the next 3
     * will be accelerometer, followed by 3 gyro, and so on.
     * 
     */
    
    // <CWA_CONVERT> <START> 
    	
    num_axes = (data_packet.numAxesBPS >> 4) & 0x0f; // 3, 6, or 9
    if (num_axes != 3 && num_axes != 6 && num_axes != 9) { fprintf(stderr, "[ERROR: num_axes not expected]"); }
    
    // sample packing format: 2 = 3x 16-bit signed or 0 = 3x 10-bit signed + 2-bit exponent
	short packing_format = data_packet.numAxesBPS  & 0x0f;
	
	// use num_axes and packing_format to determine the bytes per sample (BPS)
	short bps = 0;
	if (packing_format == 2){
		bps = 2 * num_axes;
		// e.g AX3 has 3 axes, and a sample per axes is 16 bits (2 bytes)
	}
	else if (packing_format == 0 && num_axes == 3){
		// one 32-bit value per sample that we have to unpack
		// (seems this is only possible on AX3)
		bps = 4;
	}
	
	
	// if we do have packing format 2 (1-3 sets of 3x 16-bit samples),
	// then which set of 3 in the whole set correspond to what reading?
	int accel_axis = (num_axes >= 6) ? 3 : ((num_axes >= 3) ? 0 : -1); // AX3: Axyz, AX6: Gxyz/Axyz
	int gyro_axis  = (num_axes >= 6) ? 0 : -1; // AX6: Gxyz/Axyz
	int mag_axis   = (num_axes >= 9) ? 6 : -1; // AX9: Gxyz/Axyz/Mxyz
    
    // <CWA_CONVERT> <END>
    
    if (first_block== true){ 
		
	
		printf("sample_rate: %f, sample_count: %d\n", sample_rate, sample_count);
		printf("num_axes: %d, packing_format: %d\n", num_axes, packing_format);
		
		
		// how many datapoints approx will we have
		// (7 days of recording at n Hz)
		int64_t estimated_datapoints_100hz = 7 * 24 * 60 * 60 * sample_rate; // timestamps, accelerometer
		int64_t estimated_datapoints_1hz = 7 * 24 * 60 * 60 * 1; // light, temperature
		
		// reserve space in data vectors for efficiency
		ms_timestamps.reserve(estimated_datapoints_100hz);
		data_x.reserve(estimated_datapoints_100hz);
		data_y.reserve(estimated_datapoints_100hz);
		data_z.reserve(estimated_datapoints_100hz);
		data_light.reserve(estimated_datapoints_1hz);
		data_temperature.reserve(estimated_datapoints_1hz);
		
		first_block = false;
		
    }
    
    uint32_t raw_timestamp = data_packet.timestamp;
    cwa_timestamp timestamp;
    process_cwa_timestamp(raw_timestamp, timestamp);
    //print_cwa_timestamp(timestamp);                                                    
	time_t unix_timestamp = ax3_timestamp_to_unix_epoch(timestamp);
    
    
	// <CWA_CONVERT> <START>
    float timestamp_offset = -data_packet.timestampOffset / sample_rate;

    unix_timestamp 	 += (int) floor(timestamp_offset); // ensure take negative offset into account
	timestamp_offset -= (float) floor(timestamp_offset); // ensure offset always positive
    
    double packet_start = (double) unix_timestamp + timestamp_offset;
    double packet_end 	= (double) unix_timestamp 
									+ timestamp_offset
									+ (float) sample_count / sample_rate;
									
	// <CWA_CONVERT> <END>
	
	
	// convert format unix_timestamp.ms to unix_timestamp_in_ms
	// e.g 1655288789.460000 to 1655288789460
    int64_t packet_start_ms = (int64_t) round(packet_start * 1000);
    int64_t packet_end_ms = (int64_t) round(packet_end * 1000);
    				
    //printf("%f == %ld", packet_start, packet_start_ms);
    //printf("%ld -> %ld\n\n", packet_start_ms, packet_end_ms);
	
	if (recording_start == 0) { recording_start = packet_start_ms; };
	if (packet_end_ms > recording_end) { recording_end = packet_end_ms; };
	
	// approximate all timestamps represented by this packet (approx)
    int ms_increment = 1000 / (int) sample_rate; // 1000 as 100ms
    for (int64_t t {packet_start_ms}; t < packet_end_ms; t+= ms_increment){
		ms_timestamps.push_back(t);
	}

	// <CWA_CONVERT> <START>

	// initialise accel, gyro, and mag scales (to apply to raw units)
	double accel_scale = 256;	// 1g = 256
	double gyro_scale = 2000;	// 32768 = 2000dps
	double mag_scale = 16;		// 1uT = 16
	
	// Light Scale contains Raw Light units and scales for accel & gyro (confusing)
	// 16 bits: AAAGGGLLLLLLLLLL 
	// Bottom 10 bits is last recorded light sensor value in raw units, 0 = none; 
	// top three bits are unpacked accel scale (1/2^(8+n) g); next three bits are gyro scale	(8000/2^n dps)
	uint16_t light_scale = data_packet.lightScale;
	uint16_t light_raw =  (light_scale & 0x03ff);
	accel_scale = 1 << (8 + ((light_scale >> 13) & 0x07));
	if (((light_scale >> 10) & 0x07) != 0) gyro_scale = 8000 / (1 << ((light_scale >> 10) & 0x07));

	// convert raw light units to lux
	double log10LuxTimes10Power3 = (((double)light_raw + 512.0) * 6000 / 1024);
	double lux = pow(10.0, log10LuxTimes10Power3 / 1000.0);
	data_light.push_back(lux);
	
	
	uint16_t temperature_raw = data_packet.temperature;
	double temperature_deg_C = (temperature_raw * 75.0 / 256 - 50);
	data_temperature.push_back(temperature_deg_C);
	
	
	for (int i {0}; i < sample_count; i++) 
    {

		if (bps == 4) { // each sample is 4 bytes/32bit
			
			uint32_t *samples_32bit = (uint32_t*) data_packet.rawSampleData;
			
			uint32_t sample = samples_32bit[i];
			
			int16_t x_raw = UNPACK_X(sample);
			int16_t y_raw = UNPACK_Y(sample);
			int16_t z_raw = UNPACK_Z(sample);
			
			// (OMGUI, cwa.h) "Accelerometer sample values are in units of (where g = 9.81 m/s/s) and need to have a scaling factor applied."
			
			double x_scaled = (double) x_raw / accel_scale;
			double y_scaled = (double) y_raw / accel_scale;
			double z_scaled = (double) z_raw / accel_scale;
			
			//printf("raw: %d, scaled: %f\n", x_raw, x_scaled);
			//printf("X: %f, Y: %f, Z: %f\n\n", x_scaled, y_scaled, z_scaled); 
			//printf("size: %d, capacity: %d\n", data_x.size(), data_x.capacity());
			
			data_x.push_back(x_scaled);
			data_y.push_back(y_scaled);
			data_z.push_back(z_scaled);
			
			
		}
		else { // bps is 3*2, 6*2 or 9*2 (consecutive 16-bit values)
			
			/* Warning: I don't have any data to test this */
			
			int16_t *samples_multiple16bit = (int16_t*) data_packet.rawSampleData;
			
			int16_t ax, ay, az;
			int16_t gx, gy, gz;
			int16_t mx, my, mz;
			
			if (accel_axis >= 0)
			{
				ax = samples_multiple16bit[i * bps / 2 + accel_axis + 0];
				ay = samples_multiple16bit[i * bps / 2 + accel_axis + 1];
				az = samples_multiple16bit[i * bps / 2 + accel_axis + 2];
				
				double ax_scaled = (double) ax / accel_scale;
				double ay_scaled = (double) ay / accel_scale;
				double az_scaled = (double) az / accel_scale;
				
				data_x.push_back(ax_scaled);
				data_y.push_back(ay_scaled);
				data_z.push_back(az_scaled);
			
			}
			else { ax = ay = az = 0; }
			if (gyro_axis >= 0)
			{
				gx = samples_multiple16bit[i * bps / 2 + gyro_axis + 0];
				gy = samples_multiple16bit[i * bps / 2 + gyro_axis + 1];
				gz = samples_multiple16bit[i * bps / 2 + gyro_axis + 2];
				
				double gx_scaled = (double) gx / (32768.0f/gyro_scale);
				double gy_scaled = (double) gy / (32768.0f/gyro_scale);
				double gz_scaled = (double) gz / (32768.0f/gyro_scale);
				
				data_gx.push_back(gx_scaled);
				data_gy.push_back(gy_scaled);
				data_gz.push_back(gz_scaled);
			}
			else { gx = gy = gz = 0; }
			if (mag_axis >= 0)
			{
				mx = samples_multiple16bit[i * bps / 2 + mag_axis + 0];
				my = samples_multiple16bit[i * bps / 2 + mag_axis + 1];
				mz = samples_multiple16bit[i * bps / 2 + mag_axis + 2];
				
				double mx_scaled = (double) mx / mag_scale;
				double my_scaled = (double) my / mag_scale;
				double mz_scaled = (double) mz / mag_scale;
				
				data_mx.push_back(mx_scaled);
				data_my.push_back(my_scaled);
				data_mz.push_back(mz_scaled);
			}
			else { mx = my = mz = 0; }
			
		}

		
	}
	// <CWA_CONVERT> <END>
  }

  cout << "Writing to file ..." << endl;
  
  ofstream out_file;
  out_file.open(out_filename, ios::out);
  
  /* use variable 'j' to iterate temperature/light.
			 * we only get temp/light readings once per packet,
			 * so insert on first row, AND for every n=sample_count readings.
			 * (we get sample_count readings per packet, so once these are in,
			 * new packet has begun) */
  int j {0};
  
  switch(num_axes){
		case 3:
			out_file << "timestamp,accel_x,accel_y,accel_z,light,temperature\n";
			
			for (uint64_t i {0}; i < ms_timestamps.size(); i++){
				
				if ((i == 0) || (i % sample_count == 0)){
					out_file << ms_timestamps[i] << ", " << data_x[i] << ", " << data_y[i] << ", " << data_z[i] << "," << data_light[j] << "," << data_temperature[j] << "\n";
					j+= 1;
					
				} else {
					out_file << ms_timestamps[i] << ", " << data_x[i] << ", " << data_y[i] << ", " << data_z[i] << "," << "," << "\n";
				}
				
			}
			
			break;
		
		case 6:
			out_file << "timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,light,temperature\n";
			
			for (uint64_t i {0}; i < ms_timestamps.size(); i++){
				
				if ((i == 0) || (i % sample_count == 0)){
					out_file << ms_timestamps[i] << ", " << data_gx[i] << ", " << data_gy[i] << ", " << data_gz[i] << "," 
															<< data_x[i] << ", " << data_y[i] << ", " << data_z[i] << "," 
															<< data_light[j] << "," << data_temperature[j] << "\n";
					j+= 1;
					
				} else {
					out_file << ms_timestamps[i] << ", " << data_gx[i] << ", " << data_gy[i] << ", " << data_gz[i] << "," 
															<< data_x[i] << ", " << data_y[i] << ", " << data_z[i] << "," 
															<< "," << "," << "\n";
				}
				
			}
			
			break;
			
		case 9:
			out_file << "timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,light,temperature\n";
			
			for (uint64_t i {0}; i < ms_timestamps.size(); i++){
				
				if ((i == 0) || (i % sample_count == 0)){
					out_file << ms_timestamps[i] << ", " << data_gx[i] << ", " << data_gy[i] << ", " << data_gz[i] << "," 
															<< data_x[i] << ", " << data_y[i] << ", " << data_z[i] << "," 
															<< data_mx[i] << ", " << data_my[i] << ", " << data_mz[i] << "," 
															<< data_light[j] << "," << data_temperature[j] << "\n";
					j+= 1;
					
				} else {
					out_file << ms_timestamps[i] << ", " << data_gx[i] << ", " << data_gy[i] << ", " << data_gz[i] << "," 
															<< data_x[i] << ", " << data_y[i] << ", " << data_z[i] << "," 
															<< data_mx[i] << ", " << data_my[i] << ", " << data_mz[i] << "," 
															<< "," << "," << "\n";
				}
				
			}
			
			break;
  }
  
  
  out_file.close();
  
  
}

void print_usage() {
  cout << ">>>> Axivity AX-Series Data CSV Collation Tool <<<<" << endl;
  cout << "Author: Billy C. Smith (bcsm@posteo.net)" << endl;
  cout << "Thanks & Credit to Daniel Jackson and James Christie" << endl;
  cout << "Date: September 2022" << endl;
  cout << "Usage: ./ax3-collator -i [input file].cwa -o [output file].csv" << endl;
}


int main(int argc, char* argv[]) {
	
	/* Due to licensing conflicts, all code related to Open Movement's OMGUI 
	 * (specifically 'cwa-convert')
	 * will be surrounded with <CWA-CONVERT> tags. */
	
	string in_filename;
	string out_filename;
	
	for (int i = 1; i < argc; ++i) {
		
		switch (string(argv[i])[1]) {
			
			case 'i':
				in_filename = string(argv[i+1]);
				break;
				
			case 'o':
				out_filename = string(argv[i+1]);
				break;
				
			default:
				print_usage();
				break;
		}
	}
	
	ifstream cwa_file (in_filename, ios::in | ios::binary);
	
	// TODO actually check header
	char header_buffer[1024];
	if (!cwa_file.read(header_buffer, 1024)) {
		cerr << "Error occured reading header of file, exiting." << endl;
		exit(1);
	}
	
	collate_and_convert(cwa_file, out_filename);
}
