#ifndef YESENSE_DECODER_H
#define YESENSE_DECODER_H

#include "yesense_std_out_decoder.h"

#define DATA_BUF_SIZE                   1024u

namespace yesense
{

// --------------------------------------------------------------------------------------------------------------
// structure


class yesense_decoder
{
public:
    yesense_decoder();
    ~yesense_decoder();

    int data_proc(unsigned char *data, unsigned int len, yis_out_data_t *result);

private:

#pragma pack(1)
    typedef struct
    {
        unsigned char flg   :1;
        unsigned char op    :3;
        unsigned char resv  :4;
        unsigned char data_class;
        unsigned char result;
    }serial_cmd_info_t;
#pragma pack()

    // RingBuffer环形存储区，用于存储从串口中读出的数据
    unsigned char   decode_data[DATA_BUF_SIZE];
    unsigned int    decode_buf_len;

    yis_std_out_decoder *std_out_decoder;
    serial_cmd_info_t   cmd_info;

    int clear_buf_data(unsigned int st_idx, unsigned int end_idx);
};

}

#endif

