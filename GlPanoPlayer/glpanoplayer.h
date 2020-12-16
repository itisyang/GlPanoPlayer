#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_glpanoplayer.h"

class GlPanoPlayer : public QMainWindow
{
    Q_OBJECT

public:
    GlPanoPlayer(QWidget *parent = Q_NULLPTR);

private:
    Ui::GlPanoPlayerClass ui;
};
