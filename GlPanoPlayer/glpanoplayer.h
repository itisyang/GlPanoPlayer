#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_glpanoplayer.h"

class GlPanoPlayer : public QMainWindow
{
    Q_OBJECT

public:
    GlPanoPlayer(QWidget *parent = Q_NULLPTR);

private slots:
    void on_checkbox_360_stateChanged(int state);
    void on_horizontalslider_fovy_valueChanged(int value);
private:
    Ui::GlPanoPlayerClass ui;
};
