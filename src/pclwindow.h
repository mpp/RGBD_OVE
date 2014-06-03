#ifndef PCLWINDOW_H
#define PCLWINDOW_H

#include <QMainWindow>
#include "../build/ui_pclwindow.h"

/*namespace Ui {
class Pclwindow;
}*/

class Pclwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit Pclwindow(QWidget *parent = 0);
    ~Pclwindow();

private slots:
    void on_exitAction_triggered();

    void on_openFile0Action_triggered();


private:
    Ui::Pclwindow *ui;
};

#endif // PCLWINDOW_H
