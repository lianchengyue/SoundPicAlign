#include "H264Processor.h"

#include "frontend/Resolution.h"

#include <stdlib.h>

H264Processor::H264Processor()
 : h264_buf(0),
   h264_fp(0),
   n_buffer(0)
{
    init_encoder(Resolution::get().width(), Resolution::get().height());
    init_file();
}

H264Processor::~H264Processor()
{
    close_encoder();
    close_file();
}


void H264Processor::init_encoder(int width, int height)
{
    int test;
    compress_begin(&en, width, height);
    h264_buf = (uint8_t *) malloc(sizeof(uint8_t) * width * height * 1.5);

}

void H264Processor::init_file() {
    h264_fp = fopen(h264_file_name, "wa+");
}

void H264Processor::close_encoder() {
    compress_end(&en);
    free(h264_buf);
}

void H264Processor::close_file() {
    fclose(h264_fp);
}

void H264Processor::encode_frame(unsigned char* yuv_frame, size_t yuv_length)
{
    int h264_length = 0;
    static int count = 0;
    h264_length = compress_frame(&en, -1, yuv_frame, h264_buf);
    if (h264_length > 0)
    {
        if(fwrite(h264_buf, h264_length, 1, h264_fp)>0)
        {
            printf("encode_frame num = %d\n",count++);
        }
        else
        {
            perror("encode_frame fwrite err\n");
        }

    }
}
