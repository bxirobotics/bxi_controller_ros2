#ifndef YESENSE_DECODER_COMM_H
#define YESENSE_DECODER_COMM_H

// --------------------------------------------------------------------------------------------------------------
// result of decoding
typedef enum
{
    not_surpported  = -10,
	crc_err         = -3,
	data_len_err    = -2,
	para_err        = -1,
	analysis_ok     = 0,
	analysis_done   = 1,
	no_header		= 5,
    buf_full        = 10
}analysis_res_t;

#endif

