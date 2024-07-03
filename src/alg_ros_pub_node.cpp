#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <linux/videodev2.h>
#include "alg_common/basic_types.h"
#include "jetson-utils/imageFormat.h"
#include "alg_cvt_cuda/cuda_impl.h"
#include "alg_cvt/alg_cvtColor.h"
#include "alg_isp/alg_isp_pipeline.h"

#ifdef WITH_ROS
#include "alg_rosbridge/alg_rospub.h"
#elif WITH_ROS2
#include "alg_ros2bridge/alg_ros2pub.h"
#endif

#define AILI_MAX_BUF_NUM  5
#define ALLOC_BUF_SIZE (1024 * 1024 * 32)


static uint8_t g_input_payload[ALLOC_BUF_SIZE];
static uint8_t g_output_payload[ALLOC_BUF_SIZE];
static uint8_t g_tmp_payload[ALLOC_BUF_SIZE];


#ifdef WITH_ROS
AlgRosPubNode g_rospub;
#elif WITH_ROS2
AlgRos2PubNode g_rospub;
#endif

void save_image_raw(const char *filename, void *image_ptr, uint32_t image_size)
{
    FILE *fp;
    fp = fopen(filename, "wb");
    if (fp < 0)
    {
        printf("fopen err!");
        return;
    }
    if (fwrite(image_ptr, 1, image_size, fp) < image_size)
    {
        printf("fwrite err! Out of memory \n");
    }
    fflush(fp);
    usleep(10000);
    fclose(fp);
}


static const char *level_strings[] = {"TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
FILE *g_fp;
void log_open(uint8_t id)
{
    char path[512]; // PATH_MAX is defined in limits.h
    char wd[256]; // PATH_MAX is defined in limits.h
    getcwd(wd, sizeof(wd));
    sprintf(path, "%s/v4l2_ch%02d.log", wd, id);
    g_fp = fopen(path, "a+");
    fseek(g_fp,0,SEEK_END);
}

uint64_t curtime(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000000 + tv.tv_usec;
}



void int_handler(int sig)
{
    printf("Caught signal : %d\n", sig);

#ifdef WITH_ROS
    ros::shutdown();
#elif WITH_ROS2
    rclcpp::shutdown();
#endif
    fclose(g_fp);
    /* terminate program */
    exit(sig);
}

int main(int argc, char *argv[])
{
    int status = 0;
    signal(SIGINT, int_handler);

    if (argc != 3)
    {
        printf("Usage: %s </dev/videoX> <GBRG/GRBG/RCCC>, print format detail for video device\n", argv[0]);
        return -1;
    }
    int ch_id = 0;
    sscanf(argv[1],"/dev/video%d",&ch_id);
    printf("ch_id: %d \n",ch_id);

    imageFormat input_format;
    imageFormat output_format;

    if (ch_id == 0 || ch_id == 1 || ch_id == 2 || ch_id == 3)
    {
        if(0==strncmp(argv[2],"GBRG",4))
        {
            input_format = IMAGE_BAYER_GBRG_RAW16;
        }
        else if(0==strncmp(argv[2],"GRBG",4))
        {
            input_format = IMAGE_BAYER_GRBG_RAW16;
        }
        else
        {
            printf("pattern type err:%s available type GBRG/GRBG \n",argv[2]);
            return -1;
        }
        output_format = IMAGE_RGB8_WITH_AWB;
    }
    else if (ch_id == 4)
    {
        if(0==strncmp(argv[2],"RCCC",4))
        {
            input_format = IMAGE_BAYER_RCCC_RAW16;
        }
        else
        {
            printf("pattern type err:%s available type RCCC \n",argv[2]);
            return -1; 
        }
        output_format = IMAGE_GRAY8;
    }


    log_open(ch_id);
    fprintf(g_fp,"\n\n[INFO][%lu] %s start run \n",curtime(),argv[0]);

#if 0
    /*1. 打开摄像头设备*/
    int fd = open("/dev/video0", O_RDWR);
#else
    int fd = open(argv[1], O_RDWR);
#endif
    if (fd < 0)
    {
        printf("can not open video device");
        return -1;
    }

    fprintf(g_fp,"[INFO][%lu] open video %s ok \n",curtime(),argv[1]);
	/*2 设置摄像头的属性*/
    /*2.1 查询当前摄像头驱动信息*/
    //get v4l2 capability
    struct v4l2_capability cap;
    memset(&cap, 0, sizeof(struct v4l2_capability));
    if (0 == ioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        // 驱动的名字 设备的名字 设备拥有的能力 
        printf("capability driver %s, card %s, cap 0x%x\r\n",cap.driver, cap.card, cap.capabilities);
        if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
            fprintf(stderr, "Error opening device %s: video capture not supported.\n",
                    argv[1]);
             status = -1 ;
            // goto error_buf;
        }

        if(!(cap.capabilities & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "%s does not support streaming i/o\n", argv[1]);
            status = -1 ;
            // goto error_buf;
        }
    }
    else
    {
        printf("can not get capability\n");
        status = -1 ;
        // goto error_buf;
    }

    /*2.2 查询当前摄像头支持的格式*/
	struct v4l2_fmtdesc fmtd;
    struct v4l2_frmsizeenum frmsize = {0};
    struct v4l2_frmivalenum frmival = {0};

	fmtd.index = 0;
	fmtd.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frmsize.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frmival.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	printf("video format:\n");
    while(ioctl(fd,VIDIOC_ENUM_FMT,&fmtd) == 0)
    {
		fmtd.index ++ ;
        frmsize.pixel_format = fmtd.pixelformat;
        frmival.pixel_format = fmtd.pixelformat;
		printf("format <'%c%c%c%c'--'%s'> \n",
				fmtd.pixelformat & 0xff,(fmtd.pixelformat >> 8)&0xff,
				(fmtd.pixelformat >> 16) & 0xff,(fmtd.pixelformat >> 24)&0xff,
				fmtd.description);

        /* 枚举出摄像头所支持的所有视频采集分辨率 */
        frmsize.index = 0;
        while (0 == ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) 
        {
            printf("size<%d*%d> ",
                    frmsize.discrete.width,
                    frmsize.discrete.height);
            frmsize.index++;

            /* 获取摄像头视频采集帧率 */
            frmival.index = 0;
            frmival.width = frmsize.discrete.width;
            frmival.height = frmsize.discrete.height;
            while (0 == ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival)) {

                printf("<%dfps>", frmival.discrete.denominator /
                        frmival.discrete.numerator);
                frmival.index++;
            }
            printf("\n");
        }
        printf("\n");

        fprintf(g_fp,"[INFO][%lu] video format <'%c%c%c%c'--'%s'> size<%d*%d> <%dfps> \n",curtime(),\
                fmtd.pixelformat & 0xff,(fmtd.pixelformat >> 8)&0xff,
				(fmtd.pixelformat >> 16) & 0xff,(fmtd.pixelformat >> 24)&0xff,
				fmtd.description,frmsize.discrete.width,frmsize.discrete.height,
                frmival.discrete.denominator /frmival.discrete.numerator);

    }


    /*2.3 设定摄像头格式*/
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix_mp.width = frmsize.discrete.width;
    fmt.fmt.pix_mp.height = frmsize.discrete.height;

    fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_SGBRG12;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    if (0 == ioctl(fd, VIDIOC_S_FMT, &fmt))
    {
        printf("set format ok: %d x %d plane count %d, size %d, line size %d\n",
              fmt.fmt.pix_mp.width,
              fmt.fmt.pix_mp.height,
              fmt.fmt.pix_mp.num_planes,
              fmt.fmt.pix_mp.plane_fmt[0].sizeimage,
              fmt.fmt.pix_mp.plane_fmt[0].bytesperline);
        fprintf(g_fp,"[INFO][%lu] set format ok: %d x %d plane count %d, size %d, line size %d\n",curtime(),\
              fmt.fmt.pix_mp.width,
              fmt.fmt.pix_mp.height,
              fmt.fmt.pix_mp.num_planes,
              fmt.fmt.pix_mp.plane_fmt[0].sizeimage,
              fmt.fmt.pix_mp.plane_fmt[0].bytesperline);

    }
    else
    {
        printf("can not set format\n");
        status = -1 ;
        // goto error_buf;
    }
    size_t width = fmt.fmt.pix_mp.width;
    size_t height = fmt.fmt.pix_mp.height;


	/*3 分配buf内存并mmap到进程空间*/
	/*3.1 请求分配buf*/
    struct v4l2_requestbuffers rb;
    memset(&rb, 0, sizeof(struct v4l2_requestbuffers));
    rb.count = AILI_MAX_BUF_NUM;                /*预设要申请的缓冲区数量*/
    rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;      /*视频捕获设备*/
    rb.memory = V4L2_MEMORY_MMAP;               /*支持mmap内存映射*/
    status = ioctl(fd, VIDIOC_REQBUFS, &rb);
    if(status)
    {
        printf("can not request buffers\n");
        // goto error_buf;
    }
    printf("reqbufs successful buf cnt %d ", AILI_MAX_BUF_NUM);
    fprintf(g_fp,"[INFO][%lu] reqbufs successful buf cnt %d\n",curtime(),AILI_MAX_BUF_NUM);

    /*3.2 获取缓冲区的详细信息: 地址,编号*/
    uint8_t *image_buffer[AILI_MAX_BUF_NUM];
    for(uint32_t i = 0; i < AILI_MAX_BUF_NUM; i++)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if(ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
        {
            printf("Querybuf fail\n");
            rb.count = i;
            status = -1;
			// goto error1;
        }
		/* 摄像头申请缓冲区信息: 使用mmap函数将内核的地址映射到进程空间*/
		image_buffer[i]=(uint8_t *)mmap(NULL,                   /*映射到的内存区域的起始地址，NULL表示由内核决定*/
                                        buf.length,             /*映射的内存区域*/
                                        PROT_READ|PROT_WRITE,   /*可读可写*/
                                        MAP_SHARED,             /*客一共享*/
                                        fd,                     /**/
                                        buf.m.offset); 
		if(image_buffer[i]==NULL)
		{
			printf("缓冲区映射失败!\n");
			return -6;
		}
    }

    printf("reqbufs mmap ok \n");
    fprintf(g_fp,"[INFO][%lu] reqbufs mmap ok \n",curtime());

	/*3.3 将缓冲区放入采集队列*/
    for(uint32_t i = 0; i < AILI_MAX_BUF_NUM; ++i)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(struct v4l2_buffer));
        buf.index = i;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        // buf.length = num_planes;
        if (0 != ioctl(fd, VIDIOC_QBUF, &buf))
        {
            perror("Unable to queue buffer");
            status = -1;
            // goto error2;
        }
    }

    printf("qbuf ok \n");
    fprintf(g_fp,"[INFO][%lu] qbuf ok \n",curtime());
	/*4 启动摄像头输出*/
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (0 != ioctl(fd, VIDIOC_STREAMON, &type))
    {
        perror("Unable to start capture");
        status = -1;
        // goto error2;
    }
    else
    {
        printf("start capture \n");
        fprintf(g_fp,"[INFO][%lu] start capture ... \n",curtime());
    }

	/*5 循环读取摄像头采集的数据*/
    struct pollfd fds[1];
    memset(fds, 0, sizeof(fds));
    fds[0].fd = fd;
    fds[0].events = POLLIN;

    // ros初始化
#ifdef WITH_ROS
    char pubname[256];
    sprintf(pubname,"alg_sdk_ros_publisher%02d",ch_id);
    ros::init(argc, argv, pubname, ros::init_options::NoSigintHandler);
    ros::Time::init();
    g_rospub.Init(ch_id);
#elif WITH_ROS2
    rclcpp::init(argc, argv);
    g_rospub.Init(ch_id);
#endif

    // cuda初始化
    cuda_set_device_id(0);

    size_t input_img_size = imageFormatSize(input_format, width,height);
    size_t output_img_size = imageFormatSize(output_format,width,height);

    // 申请内存
    void *input_payload;
    void *output_payload;
    size_t size = ALLOC_BUF_SIZE;
    if (!cuda_alloc_map(&input_payload,size ))
    {
        printf("failed to allocate zero-copy buffer of %zu bytes\n", size);
        return false;
    }
    if (!cuda_alloc_map(&output_payload, size))
    {
        printf("failed to allocate zero-copy buffer of %zu bytes\n", size);
        return false;
    }
    printf("cuda malloc buf successful\r\n");
    fprintf(g_fp,"[INFO][%lu] cuda malloc buf successful \n",curtime());

    uint32_t frame_index = 0;
    uint64_t timestamp = 0;
    uint32_t fps_cnt=0;
    uint64_t fps_time=0;
    while(1)
    {
        if(1 == poll(fds, 1, 2000))
        {
            /* 把buffer取出队列 */
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(struct v4l2_buffer));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            if (0 != ioctl(fd, VIDIOC_DQBUF, &buf))
            {
                perror("Unable to dequeue buffer");
                continue;;
            }
            if(ch_id == 4)
            {
                usleep(15000);
            }
            memcpy(input_payload,image_buffer[buf.index], height*width*2);

            timestamp = curtime();
            if(fps_cnt >= 30)
            {
                fps_cnt = 0;
                uint64_t offset_time_us =  timestamp - fps_time;
                double fps = (double)30000000 / offset_time_us;
                fprintf(g_fp,"[INFO][%lu] fps:%f \n",timestamp,fps);
                fps_time = timestamp;
            }
            fps_cnt++;

            // printf("ch:%d time :%lu data %d\n",ch_id, timestamp, ((uint8_t*)input_payload)[8000]);
            // char file_name[256];
            // sprintf(file_name,"/home/nvidia/bsj_workspace/alg_cv/release/test_pic_out/%05dbsj_ring_1280_1084.raw",frame_index);
            // save_image_raw(file_name, input_payload, input_img_size);
            // break;
            if (ch_id == 0 || ch_id == 1 || ch_id == 2 || ch_id == 3)
            {
                if (!cuda_cvtColor_RGB_WITHAWB(input_payload, input_format, output_payload,width,height))
                {
                    printf("CUDA -- convert image format (%d) failed! \n", input_format);
                    return 1; 
                }
                cuda_sync_device();
                uint64_t timestamp2 = curtime();
                g_rospub.PublishImage(frame_index, "bsj_ring", height,width, ALG_SDK_VIDEO_FORMAT_RGB, height*width*3, timestamp, output_payload);
                uint64_t timestamp3 = curtime();
                // printf("ch:%d cuda offset %ld ros pub :%ld\n",ch_id,timestamp2 - timestamp,timestamp3-timestamp2);
                // save_image_raw("/home/nvidia/bsj_workspace/alg_cv/release/test_pic_out/bsj_ring_1280_1084_rgb_out.raw", output_payload,output_img_size);
                // break;
            }
            else if (ch_id == 4)
            {
                if (!cuda_cvtColor_GRAY8(input_payload, input_format, output_payload, width,height))
                {
                    printf("CUDA -- convert image format (%d) failed! \n", input_format);
                    break;
                }
                cuda_sync_device();
                // save_image_raw("/home/nvidia/bsj_workspace/alg_cv/release/test_pic_out/bsj_front_640_968_gray_out.raw", output_payload,output_img_size);
                g_rospub.PublishImage(frame_index, "bsj_front",height,width, ALG_SDK_VIDEO_FORMAT_GRAY8,output_img_size, timestamp, output_payload);
            }

            frame_index++;

            /* 把buffer放入队列 */
            if (0 != ioctl(fd, VIDIOC_QBUF, &buf))
            {
                perror("Unable to queue buffer");
                continue;
            }
        }
        else
        {
            fprintf(g_fp,"[ERROR][%lu] poll err: %s\n",curtime(),strerror(errno));
            printf("poll err: %s\n", strerror(errno));
            break;
        }
    }

    cuda_free_host(input_payload);
    cuda_free_host(output_payload);
    close(fd);
    return status;
}


