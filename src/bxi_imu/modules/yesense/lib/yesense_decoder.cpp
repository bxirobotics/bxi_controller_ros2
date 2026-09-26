#include <iostream>
#include <string.h>
#include "yesense_decoder_comm.h"
#include "yesense_decoder.h"

// ===========================================================================


// ===========================================================================
namespace yesense{

yesense_decoder::yesense_decoder()
{
    memset(decode_data, 0, DATA_BUF_SIZE);
    decode_buf_len = 0u;  // 显式初始化，避免未定义行为
    std_out_decoder = new yis_std_out_decoder();

    cmd_info.flg        = 0u;
    cmd_info.result     = 0x00u;
    cmd_info.data_class = 0x00u;
}

yesense_decoder::~yesense_decoder()
{
	delete std_out_decoder;
}

int yesense_decoder::data_proc(unsigned char *data, unsigned int len, yis_out_data_t *result)
{   
    int ret = analysis_ok;

    if(len + decode_buf_len > DATA_BUF_SIZE)
    {
        // 缓冲区满：丢弃旧数据以容纳新数据，避免永久阻塞
        if(len >= DATA_BUF_SIZE)
        {
            // 新数据本身就超过缓冲区，只取最后 DATA_BUF_SIZE 字节
            memcpy(decode_data, data + (len - DATA_BUF_SIZE), DATA_BUF_SIZE);
            decode_buf_len = DATA_BUF_SIZE;
        }
        else
        {
            // 丢弃旧数据，为新数据腾入空间
            unsigned int keep = DATA_BUF_SIZE - len;
            memmove(decode_data, decode_data + (decode_buf_len - keep), keep);
            decode_buf_len = keep;
            memcpy(decode_data + decode_buf_len, data, len);
            decode_buf_len += len;
        }
        // 不返回 buf_full，继续解析
    }
    else
    {
        memcpy(decode_data + decode_buf_len, data, len);
        decode_buf_len += len;
    }

    // start decoding
    ret = std_out_decoder->data_proc(decode_data, decode_buf_len, result);
    if(no_header == ret)
    {
        if(!cmd_info.flg)
        {
            // Keep a trailing first-header byte. Serial reads may split the
            // 0x59 0x53 header between two read() calls.
            if(decode_buf_len > 0u && decode_data[decode_buf_len - 1u] == 0x59u)
            {
                decode_data[0] = decode_data[decode_buf_len - 1u];
                decode_buf_len = 1u;
            }
            else if(decode_buf_len > 0u)
            {
                clear_buf_data(0u, decode_buf_len - 1u);
            }
        }
    }
    else if(data_len_err == ret)
    {
        // msg_idx_t *msg_idx = std_out_decoder->msg_idx_obt();
        // clear_buf_data(0, msg_idx->st_idx);
    }
    else if(crc_err == ret || analysis_ok == ret)
    {
        msg_idx_t *msg_idx = std_out_decoder->msg_idx_obt();
        if(crc_err == ret)
        {
            clear_buf_data(0, msg_idx->st_idx + 1); // include header
        }
        else
        {
            clear_buf_data(0, msg_idx->end_idx);
        }
    }

    // reserved for command decoder
    // if(cmd_info.flg)
    // {

    // }

    return ret;
}

int yesense_decoder::clear_buf_data(unsigned int st_idx, unsigned int end_idx)
{
    unsigned int cnt = 0u;

    if(st_idx > end_idx)
    {
        return para_err;
    }

    cnt = end_idx - st_idx + 1u;
    if(cnt >= DATA_BUF_SIZE)
    {
        memset(decode_data, 0u, DATA_BUF_SIZE);
        decode_buf_len = 0u;     
    }
    else if(cnt > 0)
    {
        decode_buf_len -= cnt;        
        memcpy(decode_data + st_idx, decode_data + end_idx + 1u, decode_buf_len);    
    }

    return analysis_ok;
}

}
