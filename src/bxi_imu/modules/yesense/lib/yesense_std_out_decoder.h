#ifndef YESENSE_STD_OUT_DECODER_H
#define YESENSE_STD_OUT_DECODER_H

//#include <iostream>

namespace yesense
{

// --------------------------------------------------------------------------------------------------------------
// structure
#pragma pack(1)

typedef struct
{
    float x;
    float y;
    float z;
}axis_data_t;

typedef struct
{
	float pitch;			// unit: ° (deg)
	float roll;
	float yaw;
}euler_data_t;

typedef struct
{
	float q0;	
	float q1;
	float q2;
    float q3;
}quat_data_t;

typedef struct
{
	double latitude;					// unit: deg
	double longitude;					// unit: deg
	float altitude;						// unit: m
}pos_data_t;

typedef struct
{
	float vel_e;
	float vel_n;
	float vel_u;
}vel_data_t;

typedef struct
{
    unsigned short  year;
    unsigned int    month  :4;
    unsigned int    day    :5;
    unsigned int    hour   :5;
    unsigned int    min    :6;
    unsigned int    sec    :6;
    unsigned int    resv   :6;
    unsigned short  ms;
}utc_data_t;

typedef struct
{
    union 
    {
        struct
        {
            unsigned char fusion_sta:4; 
            unsigned char pos_sta   :4;
        }bit;
        unsigned char byte;
    }status;
}nav_status_t;

typedef struct
{
    unsigned short valid_flg            :1;  // indicate result is valid(1) or invalid(0)
    unsigned short sensor_temp          :1;
    unsigned short acc                  :1;   
    unsigned short gyro                 :1;
    unsigned short mag_norm             :1;
    unsigned short mag_raw              :1;
    unsigned short euler                :1;
    unsigned short quat                 :1;

    unsigned short pos                  :1;
    unsigned short utc                  :1;      
    unsigned short vel                  :1;
    unsigned short status               :1;
    unsigned short sample_timestamp     :1;          
    unsigned short dataready_timestamp  :1;  
    unsigned short pressure             :1; // reserved
    unsigned short resv                 :1;                    
}yis_content_t;

typedef struct
{
    yis_content_t   content;         
    unsigned short  tid;

	float           sensor_temp;            // data_id = 0x01, unit: °C 
    axis_data_t     acc;                    // data_id = 0x10, unit: m/s2
    axis_data_t     gyro;                   // data_id = 0x20, unit: degree/s or dps
    axis_data_t     mag_raw;                // data_id = 0x31, unit: mGauss
    axis_data_t     mag_norm;               // data_id = 0x30, unit: none
    float           pressure;               // reserved, unit: pa

    euler_data_t    euler;                  // data_id = 0x40, unit: ° (deg)
    quat_data_t     quat;                   // data_id = 0x41, unit: none

    pos_data_t      pos;                    // data_id = 0x68, unit: seen in pos_data_t
    vel_data_t      vel;                    // data_id = 0x70, unit: m/s
    utc_data_t      utc;                    // data_id = 0x50, 
 	unsigned int    sample_timestamp;		// data_id = 0x51, unit: us or 100us
	unsigned int    dataready_timestamp;	// data_id = 0x52, unit: us or 100us

 	nav_status_t    status;     // data_id = 0x80, unit: none
}yis_out_data_t;

typedef struct
{
    unsigned int st_idx;
    unsigned int end_idx;
}msg_idx_t;
#pragma pack()

class yis_std_out_decoder
{
public:
    yis_std_out_decoder();
    ~yis_std_out_decoder();

    int crc_calc(unsigned char *data, unsigned int len, unsigned short *crc);
    int data_proc(unsigned char *data, unsigned int len, yis_out_data_t *result);
    msg_idx_t *msg_idx_obt(void);

private:

#pragma pack(1)
    typedef struct
    {
        unsigned char   header1;	// 0x59
        unsigned char   header2;	// 0x53
        unsigned short  tid;		// 1 -- 60000
        unsigned char   len;		// length of payload, 0 -- 255
    }output_data_header_t;

    typedef struct
    {
        unsigned char data_id;
        unsigned char data_len;
    }payload_info_t;
#pragma pack()

    msg_idx_t msg_idx;

    int convert_data_s32(unsigned char *data, float *result, float sens, unsigned short cnt);
    int parse_data_by_id(payload_info_t *info, unsigned char *data, yis_out_data_t *result);
};

}

#endif
