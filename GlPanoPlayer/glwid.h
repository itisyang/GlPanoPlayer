#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_5_Compatibility> 
#include <QMouseEvent>

#include <thread>

extern "C"
{
#include "libavformat/avformat.h"
#include "libswscale/swscale.h"
}

class GlWid : public QOpenGLWidget, /*protected QOpenGLFunctions,*/ protected QOpenGLFunctions_4_5_Compatibility
{
    Q_OBJECT

public:
    GlWid(QWidget *parent);
    ~GlWid();

    int ThreadRead();

    int Set360(bool is_360);
    int SetFovy(int fovy);
protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);

    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

    bool is_360_;
    int fovy_;

    std::thread thread_read_;
    bool is_thread_read_running_;
};
