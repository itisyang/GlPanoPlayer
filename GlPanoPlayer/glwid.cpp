#include "glwid.h"
#include <gl/GLU.h>
#include <thread>
#include <QImage>
#include <QMutex>
#include <QDebug>

#include <opencv2/opencv.hpp>

#define PI 3.1415926

GLfloat  xangle = 0.0;    //X 旋转量
GLfloat  yangle = 0.0;    //Y 旋转量
GLfloat  zangle = 0.0;    //Z 旋转量

//交叉点的坐标
int cx = 0;
int cy = 0;

GLfloat  distance = 0.0;

GLuint  texturesArr;

int cap_H = 1;//必须大于0,且cap_H应等于cap_W
int cap_W = 1;//绘制球体时，每次增加的角度

float* verticals;
float* UV_TEX_VERTEX;

#define BALL_VIEW 0

#define MAXSIZE 10

typedef struct Vid_Frame {
    AVFrame *frame;
    int serial;
    double pts;           /* presentation timestamp for the frame */
    double duration;      /* estimated duration of the frame */
    int64_t pos;          /* byte position of the frame in the input file */

    uint8_t *buffer;

    int width;
    int height;
    AVRational sar;
} Vid_Frame;

typedef struct FrameQueue {
    Vid_Frame queue[MAXSIZE];
    int front;
    int rear;
    int size;
//     CRITICAL_SECTION cs;
    QMutex mutex;
    FrameQueue()
    {
        front = 0;
        rear = 0;
        size = 0;
    }
}FrameQueue;

void initQueue(FrameQueue *q);
int inQueue(FrameQueue *q, int num);
int outQueue(FrameQueue *q);
void deQueue(FrameQueue *q);

FrameQueue frame_queue;


void initQueue(FrameQueue *q) {
    int i;
    for (i = 0; i < MAXSIZE; i++) {
        if (!(q->queue[i].frame = av_frame_alloc()))
            return;
        q->queue[i].buffer = NULL;
    }

    q->front = 0;
    q->rear = 0;
    q->size = 0;
//     InitializeCriticalSection(&q->cs);
}

void deQueue(FrameQueue *q)
{
    free(q);
}

void getPointMatrix(GLfloat radius)
{
    verticals = new float[(180 / cap_H) * (360 / cap_W) * 6 * 3];
    UV_TEX_VERTEX = new float[(180 / cap_H) * (360 / cap_W) * 6 * 2];

    float x = 0;
    float y = 0;
    float z = 0;

    int index = 0;
    int index1 = 0;
    float r = radius;//球体半径
    double d = cap_H * PI / 180;//每次递增的弧度
    for (int i = 0; i < 180; i += cap_H) {
        double d1 = i * PI / 180;
        for (int j = 0; j < 360; j += cap_W) {
            //获得球体上切分的超小片矩形的顶点坐标（两个三角形组成，所以有六点顶点）
            double d2 = j * PI / 180;
            verticals[index++] = (float)(x + r * sin(d1 + d) * cos(d2 + d));
            verticals[index++] = (float)(y + r * cos(d1 + d));
            verticals[index++] = (float)(z + r * sin(d1 + d) * sin(d2 + d));
            //获得球体上切分的超小片三角形的纹理坐标
            UV_TEX_VERTEX[index1++] = (j + cap_W) * 1.0f / 360;
            UV_TEX_VERTEX[index1++] = (i + cap_H) * 1.0f / 180;

            verticals[index++] = (float)(x + r * sin(d1) * cos(d2));
            verticals[index++] = (float)(y + r * cos(d1));
            verticals[index++] = (float)(z + r * sin(d1) * sin(d2));

            UV_TEX_VERTEX[index1++] = j * 1.0f / 360;
            UV_TEX_VERTEX[index1++] = i * 1.0f / 180;

            verticals[index++] = (float)(x + r * sin(d1) * cos(d2 + d));
            verticals[index++] = (float)(y + r * cos(d1));
            verticals[index++] = (float)(z + r * sin(d1) * sin(d2 + d));

            UV_TEX_VERTEX[index1++] = (j + cap_W) * 1.0f / 360;
            UV_TEX_VERTEX[index1++] = i * 1.0f / 180;

            verticals[index++] = (float)(x + r * sin(d1 + d) * cos(d2 + d));
            verticals[index++] = (float)(y + r * cos(d1 + d));
            verticals[index++] = (float)(z + r * sin(d1 + d) * sin(d2 + d));

            UV_TEX_VERTEX[index1++] = (j + cap_W) * 1.0f / 360;
            UV_TEX_VERTEX[index1++] = (i + cap_H) * 1.0f / 180;

            verticals[index++] = (float)(x + r * sin(d1 + d) * cos(d2));
            verticals[index++] = (float)(y + r * cos(d1 + d));
            verticals[index++] = (float)(z + r * sin(d1 + d) * sin(d2));

            UV_TEX_VERTEX[index1++] = j * 1.0f / 360;
            UV_TEX_VERTEX[index1++] = (i + cap_H) * 1.0f / 180;

            verticals[index++] = (float)(x + r * sin(d1) * cos(d2));
            verticals[index++] = (float)(y + r * cos(d1));
            verticals[index++] = (float)(z + r * sin(d1) * sin(d2));

            UV_TEX_VERTEX[index1++] = j * 1.0f / 360;
            UV_TEX_VERTEX[index1++] = i * 1.0f / 180;
        }
    }
}


int GlWid::ThreadRead()
{
    is_thread_read_running_ = true;


    AVFormatContext	*pFormatCtx;
    int				i, videoindex;
    AVCodec			*pCodec;
    AVCodecContext	*pCodecCtx = NULL;

//     char filepath[] = "F:\\Downloads\\8k video\\4K_2D.mp4";
    char filepath[] = "rtsp://192.168.1.140/preview";

    av_register_all();
    avformat_network_init();
    pFormatCtx = avformat_alloc_context();

    AVDictionary* opts = NULL;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);//rtp over tcp，解决部分电脑上的花屏问题（vlc中设置 rtp over rtsp(tcp)）

    if (avformat_open_input(&pFormatCtx, filepath, NULL, &opts) != 0) {
        printf("Couldn't open input stream.（无法打开输入流）\n");
        return -1;
    }

    if (avformat_find_stream_info(pFormatCtx, NULL) < 0)
    {
        printf("Couldn't find stream information.（无法获取流信息）\n");
        return -1;
    }

    videoindex = -1;
    for (i = 0; i < pFormatCtx->nb_streams; i++)
        if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO)
        {
            videoindex = i;
            break;
        }
    if (videoindex == -1)
    {
        printf("Didn't find a video stream.（没有找到视频流）\n");
        return -1;
    }
    pCodecCtx = pFormatCtx->streams[videoindex]->codec;
    pCodec = avcodec_find_decoder(pCodecCtx->codec_id);
    if (pCodec == NULL)
    {
        printf("Codec not found.（没有找到解码器）\n");
        return -1;
    }
    if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0)
    {
        printf("Could not open codec.（无法打开解码器）\n");
        return -1;
    }

    AVFrame	*pFrame;
    pFrame = av_frame_alloc();
    int ret, got_picture;
    
//     AVPacket *packet = (AVPacket *)av_malloc(sizeof(AVPacket));
    AVPacket pkt, *packet = &pkt;



    struct SwsContext *img_convert_ctx;

    int index = 0;
    while (is_thread_read_running_)
    {
        int ret = av_read_frame(pFormatCtx, packet);
        if (ret < 0)
        {
            qDebug() << "av_read_frame" << ret;
            break;
        }
        

        if (packet->stream_index == videoindex)
        {
            ret = avcodec_decode_video2(pCodecCtx, pFrame, &got_picture, packet);

            av_packet_unref(packet);

            if (ret < 0)
            {
                printf("Decode Error.（解码错误）\n");
                continue;
            }
            if (got_picture)
            {
                index++;


                //去除FFmpeg警告 "deprecated pixel format used"
                switch (pFrame->format) {
                case AV_PIX_FMT_YUVJ420P:
                    pFrame->format = AV_PIX_FMT_YUV420P;
                    break;
                case AV_PIX_FMT_YUVJ422P:
                    pFrame->format = AV_PIX_FMT_YUV422P;
                    break;
                case AV_PIX_FMT_YUVJ444P:
                    pFrame->format = AV_PIX_FMT_YUV444P;
                    break;
                case AV_PIX_FMT_YUVJ440P:
                    pFrame->format = AV_PIX_FMT_YUV440P;
                    break;
                default:
                    //pixFormat = _videoStream->codec->codec->pix_fmts;
                    break;
                }

            
                while (frame_queue.size >= MAXSIZE)
                {
                    printf("size = %d   I'm WAITING ... \n", frame_queue.size);
                    Sleep(10);
                    if (!is_thread_read_running_)
                    {
                        break;
                    }
                }

//                 EnterCriticalSection(&frame_queue.cs);
                frame_queue.mutex.lock();

                if (!is_thread_read_running_)
                {
                    break;
                }

                Vid_Frame *vp;
                vp = &frame_queue.queue[frame_queue.rear];

                vp->frame->pts = pFrame->pts;

                /* alloc or resize hardware picture buffer */
                if (vp->buffer == NULL || vp->width != pFrame->width || vp->height != pFrame->height)
                {
                    if (vp->buffer != NULL)
                    {
                        av_free(vp->buffer);
                        vp->buffer = NULL;
                    }

                    int iSize = avpicture_get_size(AV_PIX_FMT_BGR24, pFrame->width, pFrame->height);
                    vp->buffer = (uint8_t *)av_mallocz(iSize);

                    vp->width = pFrame->width;
                    vp->height = pFrame->height;

                }

                avpicture_fill((AVPicture *)vp->frame, vp->buffer, AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);

                if (vp->buffer)
                {

                    img_convert_ctx = sws_getContext(vp->width, vp->height, (AVPixelFormat)pFrame->format, vp->width, vp->height,
                        AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);
                    sws_scale(img_convert_ctx, pFrame->data, pFrame->linesize, 0, vp->height, vp->frame->data, vp->frame->linesize);
                    sws_freeContext(img_convert_ctx);

                    vp->pts = pFrame->pts;
                }

                frame_queue.size++;
                frame_queue.rear = (frame_queue.rear + 1) % MAXSIZE;

//                 LeaveCriticalSection(&frame_queue.cs);
                frame_queue.mutex.unlock();

                this->update();

                //MySaveBmp("f5.bmp", vp->buffer, vp->width, vp->height);

                //int nHeight = vp->height;
                //int nWidth = vp->width;

                //Mat tmp_mat = Mat::zeros(nHeight, nWidth, CV_32FC3);

                //int k = 0;
                //for (int i = 0; i < nHeight; i++)
                //{
                //	for (int j = 0; j < nWidth; j++)
                //	{
                //		tmp_mat.at<Vec3f>(i, j)[0] = vp->buffer[k++] / 255.0f;
                //		tmp_mat.at<Vec3f>(i, j)[1] = vp->buffer[k++] / 255.0f;
                //		tmp_mat.at<Vec3f>(i, j)[2] = vp->buffer[k++] / 255.0f;
                //	}
                //}

                //imwrite("mat_Image.jpg", tmp_mat);

                //namedWindow("Marc_Antony");
                //imshow("Marc_Antony", tmp_mat);

                //waitKey(0);

            }
        }

    }



    avcodec_close(pCodecCtx);
    avformat_close_input(&pFormatCtx);

    return 0;
}

int GlWid::Set360(bool is_360)
{
    is_360_ = is_360;

    return 0;
}

int GlWid::SetFovy(int fovy)
{

}

GlWid::GlWid(QWidget *parent)
    : QOpenGLWidget(parent),
    is_360_(false)
{


}

GlWid::~GlWid()
{
    is_thread_read_running_ = false;
    if (thread_read_.joinable())
    {
        thread_read_.join();
    }
}

void GlWid::initializeGL()
{
    initializeOpenGLFunctions();

    initQueue(&frame_queue);


    glGenTextures(1, &texturesArr);    //创建纹理
    glBindTexture(GL_TEXTURE_2D, texturesArr);


//     QImage img;
//     img.load("5.png");
//     glTexImage2D(GL_TEXTURE_2D, 0, 3, img.width(), img.height(), 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, img.bits());

//     cv::Mat image = cv::imread("5.png", 1);
//     glTexImage2D(GL_TEXTURE_2D, 0, 3, image.cols, image.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, image.data);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);    //线形滤波
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);    //线形滤波

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClearDepth(1);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glDisable(GL_CULL_FACE);    //禁用裁剪

    getPointMatrix(500);

    thread_read_ = std::thread(&GlWid::ThreadRead, this);
}

void GlWid::paintGL()
{
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    if (is_360_)
    {
        if (is_360_)
        {
            glViewport(0, 0, (GLsizei)width(), (GLsizei)height());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(120, (GLfloat)width() / height(), 1.0f, 1000.0f);    //设置投影矩阵，角度
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
        }


        gluLookAt(0, 0, distance, 0, 0, 500.0, 0, 1, 0);
        //     // 	printf("distance: %f \n", distance);
        glRotatef(xangle, 1.0f, 0.0f, 0.0f);    //绕X轴旋转
        glRotatef(yangle, 0.0f, 1.0f, 0.0f);    //绕Y轴旋转
        glRotatef(zangle, 0.0f, 0.0f, 1.0f);    //绕Z轴旋转
    }

//     EnterCriticalSection(&frame_queue.cs);
    frame_queue.mutex.lock();

    int frame_w = 0;
    int frame_h = 0;

//     printf("display size = %d \n", frame_queue.size);
    if (frame_queue.size > 0)
    {
        Vid_Frame *vp = &frame_queue.queue[frame_queue.front];

        glBindTexture(GL_TEXTURE_2D, texturesArr);
        glTexImage2D(GL_TEXTURE_2D, 0, 3, vp->width, vp->height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, vp->buffer);

        frame_queue.size--;
        frame_queue.front = (frame_queue.front + 1) % MAXSIZE;

        frame_w = vp->width;
        frame_h = vp->height;
    }

//     LeaveCriticalSection(&frame_queue.cs);
    frame_queue.mutex.unlock();


    glBindTexture(GL_TEXTURE_2D, texturesArr);

//     QImage img;
//     img.load("5.png");
//     glTexImage2D(GL_TEXTURE_2D, 0, 3, img.width(), img.height(), 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, img.bits());

//     cv::Mat image = cv::imread("5.png", 1);
//     glTexImage2D(GL_TEXTURE_2D, 0, 3, image.cols, image.rows, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, image.data);

    if (is_360_)
    {


        //glColor3f(1.0, 0.0, 0.0);
        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);

        glVertexPointer(3, GL_FLOAT, 0, verticals);
        glTexCoordPointer(2, GL_FLOAT, 0, UV_TEX_VERTEX);

        glPushMatrix();
        glDrawArrays(GL_TRIANGLES, 0, (180 / cap_H) * (360 / cap_W) * 6);

        glPopMatrix();
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    }
    else
    {
        const GLfloat bgTextureVertices[] = { 0, 0, frame_w, 0, 0, frame_h, frame_w, frame_h };
        const GLfloat bgTextureCoords[] = { 1, 0, 1, 1, 0, 0, 0, 1 };
        const GLfloat proj[] = { 0, -2.f / frame_w, 0, 0, -2.f / frame_h, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };

        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(proj);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glVertexPointer(2, GL_FLOAT, 0, bgTextureVertices);
        glTexCoordPointer(2, GL_FLOAT, 0, bgTextureCoords);
        glColor4f(1, 1, 1, 1);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        glDisableClientState(GL_VERTEX_ARRAY);  // disable vertex arrays
    }
   
    glFlush();

//     av_usleep(25000);
}

void GlWid::resizeGL(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //glOrtho(-250.0, 250, -250.0, 250, -500, 500);
    //glFrustum(-250.0, 250, -250.0, 250, -5, -500);
    gluPerspective(120, (GLfloat)w / h, 1.0f, 1000.0f);    //设置投影矩阵，角度
//     gluPerspective(45, (GLfloat)w / h, 1.0f, 1000.0f);    //设置投影矩阵，角度
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GlWid::mousePressEvent(QMouseEvent *event)
{
    cx = event->pos().x();
    cy = event->pos().y();
}

void GlWid::mouseReleaseEvent(QMouseEvent *event)
{

}

void GlWid::mouseMoveEvent(QMouseEvent *event)
{
    int x = event->pos().x();
    int y = event->pos().y();

    float offset = 0.18;
    //计算拖动后的偏移量，然后进行xy叠加减
    yangle -= ((x - cx) * offset);

    if (y > cy) {//往下拉
        xangle += ((y - cy) * offset);
    }
    else if (y < cy) {//往上拉
        xangle += ((y - cy) * offset);
    }

    if (is_360_)
    {
        update();
    }


    //保存好当前拖放后光标坐标点
    cx = x;
    cy = y;
}

void GlWid::wheelEvent(QWheelEvent *event)
{
    if (event->delta() > 0)
    {
        // 当滚轮远离使用者时,进行放大
        distance -= 8;
    }
    else
    {
        // 当滚轮向使用者方向旋转时,进行缩小
        distance += 8;
    }

    if (is_360_)
    {
        update();
    }
}
