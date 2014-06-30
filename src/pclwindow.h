#ifndef PCLWINDOW_H
#define PCLWINDOW_H

#include <QMainWindow>

#include <QFileDialog>
#include <QMessageBox>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <vtkRenderWindow.h>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "../build/ui_pclwindow.h"
#include "cloudinteraction.h"

class Pclwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit Pclwindow(QWidget *parent = 0);
    ~Pclwindow();

    Eigen::Matrix4f getTransformMatrix(const int id);

    void loadCloud0(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void loadCloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void loadCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void loadCloud3(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);


private slots:
    void on_exitAction_triggered();

    void on_openFile0Action_triggered();

    void on_openFile1Action_triggered();

    void on_openFile2Action_triggered();

    void on_openFile3Action_triggered();

    void on_cloud0Radio_toggled(bool checked);

    void on_cloud1Radio_toggled(bool checked);

    void on_cloud2Radio_toggled(bool checked);

    void on_cloud3Radio_toggled(bool checked);

    void on_xTIncButton_clicked();

    void on_yTIncButton_clicked();

    void on_zTIncButton_clicked();

    void on_xTDecButton_clicked();

    void on_yTDecButton_clicked();

    void on_zTDecButton_clicked();

    void on_xRIncButton_clicked();

    void on_xRDecButton_clicked();

    void on_yRIncButton_clicked();

    void on_yRDecButton_clicked();

    void on_zRIncButton_clicked();

    void on_zRDecButton_clicked();

    void on_computeBBButton_clicked();

    void on_translationStepInput_textChanged(const QString &arg1);

    void on_rotationStepInput_textChanged(const QString &arg1);

private:
    void enableButtons();

    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudToEdit();

    void updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudToEdit);

    void updateTransform(boost::shared_ptr<Eigen::Matrix4f> &transformMatrix);

    void loadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int id);

private:
    Ui::Pclwindow *ui;

    pcl::PointCloud<pcl::PointXYZ>::Ptr
        c_0_,
        c_1_,
        c_2_,
        c_3_;

    boost::shared_ptr<Eigen::Matrix4f>
        t_0_,
        t_1_,
        t_2_,
        t_3_;

    float
        translation_step_,
        rotation_step_;

    int cloud_to_edit_;
};

#endif // PCLWINDOW_H
