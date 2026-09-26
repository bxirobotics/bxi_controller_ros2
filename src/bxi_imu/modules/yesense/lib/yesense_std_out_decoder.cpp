#include <stdio.h>
#include <string.h>
#include "yesense_decoder_comm.h"
#include "yesense_std_out_decoder.h"

// ===========================================================================
#define PROTOCOL_FIRST_BYTE			                (unsigned char)0x59
#define PROTOCOL_SECOND_BYTE		                (unsigned char)0x53

#define PROTOCOL_FIRST_BYTE_POS 		            0
#define PROTOCOL_SECOND_BYTE_POS		            1

#define PROTOCOL_TID_LEN				            2u
#define PROTOCOL_MIN_LEN				            7u	// header(2B) + tid(2B) + len(1B) + CK1(1B) + CK2(1B)

#define CRC_CALC_START_POS				            2
#define CRC_CALC_LEN(payload_len)		            ((payload_len) + 3)	// 3 = tid(2B) + len(1B)
#define PROTOCOL_CRC_DATA_POS(payload_len)			(CRC_CALC_START_POS + CRC_CALC_LEN(payload_len))

#define PAYLOAD_POS						            5

//
#define DATA_OFFSET_TO_DATA_ID                      2u

// data id define
#define SENSOR_TEMP_ID                              (unsigned char)0x01
#define ACCEL_ID				                    (unsigned char)0x10
#define GYRO_ID				                        (unsigned char)0x20
#define MAGNETIC_NORM_ID				            (unsigned char)0x30     
#define RAW_MAGNETIC_ID			                    (unsigned char)0x31     
#define EULER_ID				                    (unsigned char)0x40
#define QUATERNION_ID			                    (unsigned char)0x41
#define UTC_ID					                    (unsigned char)0x50
#define SAMPLE_TIMESTAMP_ID		                    (unsigned char)0x51
#define DATA_READY_TIMESTAMP_ID	                    (unsigned char)0x52
#define LOCATION_ID				                    (unsigned char)0x68     // high precision location
#define SPEED_ID				                    (unsigned char)0x70
#define STATUS_ID				                    (unsigned char)0x80 	// nav status

// length for specific data id
#define SENSOR_TEMP_DATA_LEN                        (unsigned char)2
#define ACCEL_DATA_LEN					            (unsigned char)12
#define GYRO_DATA_LEN					            (unsigned char)12
#define MAGNETIC_NORM_DATA_LEN				        (unsigned char)12
#define MAGNETIC_RAW_DATA_LEN			            (unsigned char)12
#define EULER_DATA_LEN					            (unsigned char)12
#define QUATERNION_DATA_LEN				            (unsigned char)16
#define UTC_DATA_LEN					            (unsigned char)11
#define SAMPLE_TIMESTAMP_DATA_LEN		            (unsigned char)4
#define DATA_READY_TIMESTAMP_DATA_LEN	            (unsigned char)4
#define LOCATION_DATA_LEN				            (unsigned char)20   // high precision location data len
#define SPEED_DATA_LEN          		            (unsigned char)12
#define STATUS_DATA_LEN					            (unsigned char)1    // nav status

// factor for sensor data
#define SENSOR_TEMP_FACTOR                          0.01f

#define ACC_DATA_FACTOR			                    0.000001f
#define GYRO_DATA_FACTOR			                0.000001f
#define MAG_NORM_DATA_FACTOR			            0.000001f
#define MAG_RAW_DATA_FACTOR			                0.001f

#define EULER_DATA_FACTOR			                0.000001f
#define QUATENION_DATA_FACTOR			            0.000001f

#define HIGH_RES_LONG_LAT_DATA_FACTOR               0.0000000001
#define ALT_DATA_FACTOR				                0.001f
#define VEL_DATA_FACTOR			                    0.001f

// ===========================================================================
namespace yesense
{

yis_std_out_decoder::yis_std_out_decoder()
{
    msg_idx.st_idx  = 0u;
    msg_idx.end_idx = 0u;
}

yis_std_out_decoder::~yis_std_out_decoder()
{

}

int yis_std_out_decoder::crc_calc(unsigned char *data, unsigned int len, unsigned short *crc)
{
    unsigned int i = 0u;
    unsigned char ck1 = 0u, ck2 = 0u;

    if(NULL == data || 0u == len)
    {
        return para_err;
    }

    for(i = 0u; i < len; i++)
    {
        ck1 += data[i];
        ck2 += ck1;       
    }

    *crc = (ck2 << 8) | ck1;

    return analysis_ok;
}

int yis_std_out_decoder::data_proc(unsigned char *data, unsigned int len, yis_out_data_t *result)
{   
    payload_info_t *tlv = NULL;
    int ret = 0;    
    unsigned int cnt = len;
    unsigned int i   = 0u;
    unsigned char *ptr = NULL;
    unsigned short crc = 0u;

    if(NULL == data || 0u == len || NULL == result)
    {
        return para_err;
    }

    ptr = data;
    while(cnt > 1u)
    {
        // judge message header
        if(PROTOCOL_FIRST_BYTE == ptr[i] && PROTOCOL_SECOND_BYTE == ptr[i + 1])
        {
            break;
        }
        else
        {
            i++;
            cnt--;
        }
    }
    
    if(cnt <= 1u)
    {
        return no_header;      
    }

    msg_idx.st_idx  = i;

    // check message length
    if(cnt <= PROTOCOL_MIN_LEN)
    {
        return data_len_err;
    }

    // Read header fields safely via memcpy (avoid unaligned access)
    unsigned short header_tid;
    unsigned char header_len = ptr[i + 4];
    memcpy(&header_tid, ptr + i + 2, sizeof(header_tid));

    if(header_len + PROTOCOL_MIN_LEN > cnt)
    {
        return data_len_err;
    }

    // check message checksum
    crc_calc(ptr + i + CRC_CALC_START_POS, CRC_CALC_LEN(header_len), &crc);
    unsigned short crc_recv;
    memcpy(&crc_recv, ptr + i + PROTOCOL_CRC_DATA_POS(header_len), sizeof(crc_recv));
    if(crc != crc_recv)
    {
        return crc_err;
    }

    result->tid     = header_tid;
    msg_idx.end_idx = i + header_len + PROTOCOL_MIN_LEN - 1;

#ifdef __USER_DEBUG
    printf("frame len %d\n", header_len + PROTOCOL_MIN_LEN); 
    for(unsigned int n = 0u; n < header_len + PROTOCOL_MIN_LEN; n++)
    {
        printf("%02x ", ptr[i + n]);        
    }
    printf("\n");    
#endif

    // decode by data_id
    i += PAYLOAD_POS;
    cnt = header_len;
    while(cnt)
    {
        // 边界安全检查：至少需要 2 字节（data_id + data_len）
        if(cnt < 2u)
        {
            break;
        }
        tlv = (payload_info_t *)(ptr + i);
        // 防止 data_len 超出剩余 payload 导致下溢/越界
        if((unsigned int)(tlv->data_len) + 2u > cnt)
        {
            break;
        }
        ret = parse_data_by_id(tlv, ptr + i + DATA_OFFSET_TO_DATA_ID, result); 
#ifdef __USER_DEBUG        
        printf("ret = %d, i = %d, data id %02x, data len %d\n", ret, i, tlv->data_id, tlv->data_len);        
#endif        
        if(analysis_ok != ret)
        {
            i++;
            cnt--;
        }   
        else
        {
            i += tlv->data_len + 2u;
            cnt -= tlv->data_len + 2u;
        }
    } 

    return analysis_ok;
}

msg_idx_t *yis_std_out_decoder::msg_idx_obt(void)
{
    return &msg_idx;
}

int yis_std_out_decoder::convert_data_s32(unsigned char *data, float *result, float sens, unsigned short cnt)
{
    if(NULL == data || NULL == result)
    {
        return para_err;       
    }

    for(unsigned short i = 0; i < cnt; i++)
    {
        int tmp;
        memcpy(&tmp, data + i * sizeof(int), sizeof(int));
        result[i] = tmp * sens;
    }   

    return analysis_ok; 
}

int yis_std_out_decoder::parse_data_by_id(payload_info_t *info, unsigned char *data, yis_out_data_t *result)
{
    int ret = analysis_ok;
    unsigned char *ptr = NULL;

    if(NULL == info || NULL == result || NULL == data)
    {
        return para_err;
    }

    ptr = data;

#ifdef __USER_DEBUG    
    for(unsigned short i = 0u; i < info->data_len; i++)
    {
        printf("%02x ", ptr[i]);        
    }
#endif

	switch(info->data_id)
	{
		case SENSOR_TEMP_ID:
		{
			if(SENSOR_TEMP_DATA_LEN == info->data_len)
			{
				short temp_raw;
				memcpy(&temp_raw, ptr, sizeof(temp_raw));
				result->sensor_temp         = temp_raw * SENSOR_TEMP_FACTOR;
                result->content.valid_flg   = 1u;
                result->content.sensor_temp = 1u;
			}
			else
			{
				ret = data_len_err;
                result->content.sensor_temp = 0u;                   
			}
		}
		break;

		case ACCEL_ID:
		{
			if(ACCEL_DATA_LEN == info->data_len)
			{
                convert_data_s32(ptr, &result->acc.x, ACC_DATA_FACTOR, 3u);
                result->content.valid_flg   = 1u;
                result->content.acc         = 1u;               
			}
			else
			{
				ret = data_len_err;
                result->content.acc = 0u;                   
			}
		}
		break;

		case GYRO_ID:
		{
			if(GYRO_DATA_LEN == info->data_len)
			{
                convert_data_s32(ptr, &result->gyro.x, GYRO_DATA_FACTOR, 3u);                
                result->content.valid_flg   = 1u;
                result->content.gyro        = 1u;                
			}
			else
			{
				ret = data_len_err;
                result->content.gyro = 0u;                   
			}
		}
		break;

		case MAGNETIC_NORM_ID:
		{
			if(MAGNETIC_NORM_DATA_LEN == info->data_len)
			{
                convert_data_s32(ptr, &result->mag_norm.x, MAG_NORM_DATA_FACTOR, 3u);                
                result->content.valid_flg   = 1u;
                result->content.mag_norm    = 1u;                     
			}
			else
			{
				ret = data_len_err;
                result->content.mag_norm = 0u;                   
			}
		}
		break;

		case RAW_MAGNETIC_ID:
		{
			if(MAGNETIC_RAW_DATA_LEN == info->data_len)
			{
                convert_data_s32(ptr, &result->mag_raw.x, MAG_RAW_DATA_FACTOR, 3u);                
                result->content.valid_flg   = 1u;
                result->content.mag_raw     = 1u;                  
			}
			else
			{
				ret = data_len_err;
                result->content.mag_raw = 0u;                   
			}
		}
		break;

		case EULER_ID:
		{			
			if(EULER_DATA_LEN == info->data_len)
			{
                convert_data_s32(ptr, &result->euler.pitch, EULER_DATA_FACTOR, 3u);                      
                result->content.valid_flg   = 1u;  
                result->content.euler       = 1u;                 
			}
			else
			{
				ret = data_len_err;
                result->content.euler = 0u;                   
			}
		}
		break;

		case QUATERNION_ID:
		{
			if(QUATERNION_DATA_LEN == info->data_len)
			{
                convert_data_s32(ptr, &result->quat.q0, QUATENION_DATA_FACTOR, 4u);      
                result->content.valid_flg   = 1u;  
                result->content.quat        = 1u;             
			}
			else
			{
				ret = data_len_err;
                result->content.quat = 0u;                   
			}
		}
		break;

		case LOCATION_ID:
		{
			if(LOCATION_DATA_LEN == info->data_len)
			{
				long long int lat_raw, lon_raw;
				int alt_raw;
				memcpy(&lat_raw, ptr, sizeof(lat_raw));
				memcpy(&lon_raw, ptr + 8, sizeof(lon_raw));
				memcpy(&alt_raw, ptr + 16, sizeof(alt_raw));
				result->pos.latitude  = lat_raw * HIGH_RES_LONG_LAT_DATA_FACTOR;
				result->pos.longitude = lon_raw * HIGH_RES_LONG_LAT_DATA_FACTOR;
				result->pos.altitude  = alt_raw * ALT_DATA_FACTOR;
                result->content.valid_flg   = 1u;
                result->content.pos         = 1u;
			}
			else
			{
				ret = data_len_err;
                result->content.pos = 0u;               
			}
		}
		break;
		
		case SPEED_ID:
		{
			if(SPEED_DATA_LEN == info->data_len)
			{
                convert_data_s32(ptr, &result->vel.vel_e, VEL_DATA_FACTOR, 3u);                 
                result->content.valid_flg   = 1u;       
                result->content.vel         = 1u;         
			}
			else
			{
				ret = data_len_err;
                result->content.vel = 0u;                 
			}
		}
		break;

		case STATUS_ID:
		{
			if(STATUS_DATA_LEN == info->data_len)
			{
				result->status.status.byte  = *ptr;
                result->content.valid_flg   = 1u;    
                result->content.status      = 1u;
			}
			else
			{
				ret = data_len_err;
                result->content.status = 0u;                    
			}
		}
		break;
		
		case SAMPLE_TIMESTAMP_ID:
		{
			if(SAMPLE_TIMESTAMP_DATA_LEN == info->data_len)
			{
				unsigned int ts_raw;
				memcpy(&ts_raw, ptr, sizeof(ts_raw));
				result->sample_timestamp            = ts_raw;
                result->content.valid_flg           = 1u;    
                result->content.sample_timestamp    = 1u;                                   
			}
			else
			{
				ret = data_len_err;
                result->content.status = 0u;                   
			}
		}
		break;

		case DATA_READY_TIMESTAMP_ID:
		{
			if(DATA_READY_TIMESTAMP_DATA_LEN == info->data_len)
			{
				unsigned int dr_ts_raw;
				memcpy(&dr_ts_raw, ptr, sizeof(dr_ts_raw));
				result->dataready_timestamp         = dr_ts_raw;
                result->content.valid_flg           = 1u;     
                result->content.dataready_timestamp = 1u;              
			}
			else
			{
				ret = data_len_err;
                result->content.status = 0u;                   
			}
		}
		break;		
		
		default:
        {
            ret = not_surpported;
        }
		break;
	}

    if(analysis_ok != ret)
    {
        result->content.valid_flg = 0u;
    }

	return ret;
}

}
