#ifndef H264PROCESSOR_H
#define H264PROCESSOR_H

#include "../utils/H264Encoder.h"
#include <assert.h>

typedef struct{
    void *start;
    int length;
}BUFTYPE;

class H264Processor
{
  public:
    H264Processor();
    virtual ~H264Processor();

    void init_encoder(int width, int height);
    void init_file();
    void close_encoder();
    void close_file();
    void encode_frame(unsigned char* yuv_frame, size_t yuv_length);

  private:
    typedef struct{
        void *start;
        int length;
    }BUFTYPE;

    char h264_file_name[20] = "test.h264";
    Encoder en;
    uint8_t *h264_buf;
    FILE *h264_fp;

    BUFTYPE *usr_buf;
    /*static */unsigned int n_buffer;
};

#endif /* H264PROCESSOR_H */
