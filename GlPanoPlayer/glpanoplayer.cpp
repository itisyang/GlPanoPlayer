#include "glpanoplayer.h"

#pragma execution_character_set("utf-8")

GlPanoPlayer::GlPanoPlayer(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
}

void GlPanoPlayer::on_checkbox_360_stateChanged(int state)
{
    if (Qt::Unchecked == state)
    {
        ui.checkbox_360->setText("360²¥·Å");
        ui.openGLWidget->Set360(false);
    }
    else
    {
        ui.checkbox_360->setText("Æ½ÆÌ²¥·Å");
        ui.openGLWidget->Set360(true);
    }

}

void GlPanoPlayer::on_horizontalslider_fovy_valueChanged(int value)
{
    ui.openGLWidget->SetFovy(value);
}
