#include <boost/make_shared.hpp>
#include "pclwindow.h"

pcl::visualization::PCLVisualizer pviz ("test_viz", false);


Pclwindow::Pclwindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Pclwindow)
{
    ui->setupUi(this);

    vtkSmartPointer<vtkRenderWindow> renderWindow = pviz.getRenderWindow();
    ui->widget->SetRenderWindow (renderWindow);

    pviz.setupInteractor (ui->widget->GetInteractor (), ui->widget->GetRenderWindow ());
    pviz.getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

    pviz.setBackgroundColor(0, 0, 0.1);
    pviz.addCoordinateSystem (1.0);

    ui->widget->show();
    ui->widget->setDisabled(true);

    ui->translationStepInput->setText(std::to_string(0.001).c_str());
    ui->rotationStepInput->setText(std::to_string(0.01).c_str());

    translation_step_ = 0.001;
    rotation_step_ = 0.01;

    cloud_to_edit_ = 0;

    Eigen::Matrix4f mat = clinter::identity();

    t_0_ = boost::make_shared<Eigen::Matrix4f>(mat);
    t_1_ = boost::make_shared<Eigen::Matrix4f>(mat);
    t_2_ = boost::make_shared<Eigen::Matrix4f>(mat);
    t_3_ = boost::make_shared<Eigen::Matrix4f>(mat);
}

Pclwindow::~Pclwindow()
{
    delete ui;
}

void Pclwindow::loadCloud0(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    loadCloud(cloud, 0);
}

void Pclwindow::loadCloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    loadCloud(cloud, 1);
}

void Pclwindow::loadCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    loadCloud(cloud, 2);
}

void Pclwindow::loadCloud3(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    loadCloud(cloud, 3);
}

void Pclwindow::loadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int id)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToLoad;

    std::string cloudName = "";
    int r,g,b;
    switch (id) {
        case 0:
            cloudToLoad = c_0_;
            ui->cloud0Radio->setEnabled(true);
            ui->cloud0Radio->setChecked(true);
            cloudName = "cloud0";
            r = 255; g = 100; b = 255;
            break;
        case 1:
            cloudToLoad = c_1_;
            ui->cloud1Radio->setEnabled(true);
            ui->cloud1Radio->setChecked(true);
            cloudName = "cloud1";
            r = 100; g = 100; b = 255;
            break;
        case 2:
            cloudToLoad = c_2_;
            ui->cloud2Radio->setEnabled(true);
            ui->cloud2Radio->setChecked(true);
            cloudName = "cloud2";
            r = 255; g = 100; b = 100;
            break;
        case 3:
        default:
            cloudToLoad = c_3_;
            ui->cloud3Radio->setEnabled(true);
            ui->cloud3Radio->setChecked(true);
            cloudName = "cloud3";
            r = 100; g = 255; b = 100;
            break;
    }

    cloudToLoad = cloud;

    enableButtons();

    if (cloudToLoad->size() > 0)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloudToLoad, r, g, b);
        pviz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        pviz.addPointCloud<pcl::PointXYZ>(cloudToLoad, single_color, cloudName);
    }
}

void Pclwindow::on_computeBBButton_clicked()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr global(new pcl::PointCloud<pcl::PointXYZ>());
    if (c_0_)
        (*global) += (*c_0_);
    if (c_1_)
        (*global) += (*c_1_);
    if (c_2_)
        (*global) += (*c_2_);
    if (c_3_)
        (*global) += (*c_3_);

    if (global->points.size() <= 0)
    {
        return;
    }

    BoundingBox bb;

    clinter::computeBB(global, bb);

    pviz.removeShape("bb");
    pviz.addCube(bb.min_pt.x, bb.max_pt.x,
                 bb.min_pt.y, bb.max_pt.y,
                 bb.min_pt.z, bb.max_pt.z,
                 1.0,1.0,1.0,
                 "bb");

    std::cout << "(x,y,z): (" << bb.max_pt.x - bb.min_pt.x
              <<  ", " << bb.max_pt.y - bb.min_pt.y
              <<  ", " << bb.max_pt.z - bb.min_pt.z << ")" << std::endl;
}

void Pclwindow::on_exitAction_triggered()
{
    this->close();
}

void Pclwindow::enableButtons()
{
    ui->xRDecButton->setEnabled(true);
    ui->xTDecButton->setEnabled(true);
    ui->xRIncButton->setEnabled(true);
    ui->xTIncButton->setEnabled(true);

    ui->yRDecButton->setEnabled(true);
    ui->yTDecButton->setEnabled(true);
    ui->yRIncButton->setEnabled(true);
    ui->yTIncButton->setEnabled(true);

    ui->zRDecButton->setEnabled(true);
    ui->zTDecButton->setEnabled(true);
    ui->zRIncButton->setEnabled(true);
    ui->zTIncButton->setEnabled(true);

    ui->translationStepInput->setEnabled(true);
    ui->rotationStepInput->setEnabled(true);

    ui->computeBBButton->setEnabled(true);
}

void Pclwindow::on_openFile0Action_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.pcd)"));
    if (fileName != "")
    {
        pviz.removePointCloud("cloud0");
        ui->widget->setDisabled(true);
        ui->menuBar->setDisabled(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
        c_0_ = cloud_xyz;
        boost::thread t(boost::bind(pcl::io::loadPCDFile<pcl::PointXYZ>, fileName.toStdString(), boost::ref(*c_0_)));
        t.join();

        ui->widget->setEnabled(true);
        ui->menuBar->setEnabled(true);

        ui->cloud0Radio->setEnabled(true);
        ui->cloud0Radio->setChecked(true);
        enableButtons();

        if (cloud_xyz->size() > 0)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(c_0_, 255, 100, 255);
            pviz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud0");
            pviz.addPointCloud<pcl::PointXYZ>(c_0_, single_color, "cloud0");
        }

    };
}

void Pclwindow::on_openFile1Action_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.pcd)"));
    if (fileName != "")
    {
        pviz.removePointCloud("cloud1");
        ui->widget->setDisabled(true);
        ui->menuBar->setDisabled(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
        c_1_ = cloud_xyz;
        boost::thread t(boost::bind(pcl::io::loadPCDFile<pcl::PointXYZ>, fileName.toStdString(), boost::ref(*c_1_)));
        t.join();

        ui->widget->setEnabled(true);
        ui->menuBar->setEnabled(true);

        ui->cloud1Radio->setEnabled(true);
        ui->cloud1Radio->setChecked(true);
        enableButtons();

        if (cloud_xyz->size() > 0)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(c_1_, 100, 100, 255);
            pviz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
            pviz.addPointCloud<pcl::PointXYZ>(c_1_, single_color, "cloud1");
        }

    };
}

void Pclwindow::on_openFile2Action_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.pcd)"));
    if (fileName != "")
    {
        pviz.removePointCloud("cloud2");
        ui->widget->setDisabled(true);
        ui->menuBar->setDisabled(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
        c_2_ = cloud_xyz;
        boost::thread t(boost::bind(pcl::io::loadPCDFile<pcl::PointXYZ>, fileName.toStdString(), boost::ref(*c_2_)));
        t.join();

        ui->widget->setEnabled(true);
        ui->menuBar->setEnabled(true);

        ui->cloud2Radio->setEnabled(true);
        ui->cloud2Radio->setChecked(true);
        enableButtons();

        if (cloud_xyz->size() > 0)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(c_2_, 255, 100, 100);
            pviz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
            pviz.addPointCloud<pcl::PointXYZ>(c_2_, single_color, "cloud2");
        }

    };
}

void Pclwindow::on_openFile3Action_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), "", tr("Files (*.pcd)"));
    if (fileName != "")
    {
        pviz.removePointCloud("cloud3");
        ui->widget->setDisabled(true);
        ui->menuBar->setDisabled(true);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
        c_3_ = cloud_xyz;
        boost::thread t(boost::bind(pcl::io::loadPCDFile<pcl::PointXYZ>, fileName.toStdString(), boost::ref(*c_3_)));
        t.join();

        ui->widget->setEnabled(true);
        ui->menuBar->setEnabled(true);

        ui->cloud3Radio->setEnabled(true);
        ui->cloud3Radio->setChecked(true);
        enableButtons();

        if (cloud_xyz->size() > 0)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(c_3_, 100, 255, 100);
            pviz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud3");
            pviz.addPointCloud<pcl::PointXYZ>(c_3_, single_color, "cloud3");
        }

    };
}

void Pclwindow::on_cloud0Radio_toggled(bool checked)
{
    if (checked)
    {
        cloud_to_edit_ = 0;
    }
}

void Pclwindow::on_cloud1Radio_toggled(bool checked)
{
    if (checked)
    {
        cloud_to_edit_ = 1;
    }
}

void Pclwindow::on_cloud2Radio_toggled(bool checked)
{
    if (checked)
    {
        cloud_to_edit_ = 2;
    }
}

void Pclwindow::on_cloud3Radio_toggled(bool checked)
{
    if (checked)
    {
        cloud_to_edit_ = 3;
    }
}

void Pclwindow::on_xTIncButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f> transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::x_t_inc_callback(cloudToEdit(), transform, translation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_yTIncButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::y_t_inc_callback(cloudToEdit(), transform, translation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_zTIncButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::z_t_inc_callback(cloudToEdit(), transform, translation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_xTDecButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::x_t_dec_callback(cloudToEdit(), transform, translation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_yTDecButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::y_t_dec_callback(cloudToEdit(), transform, translation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_zTDecButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::z_t_dec_callback(cloudToEdit(), transform, translation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_xRIncButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::x_r_inc_callback(cloudToEdit(), transform, rotation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_xRDecButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::x_r_dec_callback(cloudToEdit(), transform, rotation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_yRIncButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::y_r_inc_callback(cloudToEdit(), transform, rotation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_yRDecButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::y_r_dec_callback(cloudToEdit(), transform, rotation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_zRIncButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::z_r_inc_callback(cloudToEdit(), transform, rotation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

void Pclwindow::on_zRDecButton_clicked()
{
    boost::shared_ptr<Eigen::Matrix4f>  transform = boost::make_shared<Eigen::Matrix4f>(clinter::identity());
    clinter::z_r_dec_callback(cloudToEdit(), transform, rotation_step_);
    updateCloud(cloudToEdit());
    updateTransform(transform);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr &Pclwindow::cloudToEdit()
{
    if (0 == cloud_to_edit_)
        return c_0_;
    if (1 == cloud_to_edit_)
        return c_1_;
    if (2 == cloud_to_edit_)
        return c_2_;
    if (3 == cloud_to_edit_)
        return c_3_;
}

void Pclwindow::updateTransform(boost::shared_ptr<Eigen::Matrix4f> &transformMatrix)
{
    // Take the right transform matrix to update
    boost::shared_ptr<Eigen::Matrix4f> toUpdate;

    switch (cloud_to_edit_) {
        case 0:
            toUpdate = t_0_;
            break;
        case 1:
            toUpdate = t_1_;
            break;
        case 2:
            toUpdate = t_2_;
            break;
        case 3:
        default:
            toUpdate = t_3_;
            break;
    }

    // Update the transform matrix with a right-multiplication with the last transform applied
    (*toUpdate) = (*toUpdate) * (*transformMatrix);
}

void Pclwindow::updateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudToEdit)
{
    int r,g,b;
    std::string cloudName;
    switch (cloud_to_edit_) {
    case 0:
        r = 255;
        g = 100;
        b = 255;
        cloudName = "cloud0";
        break;
    case 1:
        r = 100;
        g = 100;
        b = 255;
        cloudName = "cloud1";
        break;
    case 2:
        r = 255;
        g = 100;
        b = 100;
        cloudName = "cloud2";
        break;
    case 3:
    default:
        r = 100;
        g = 255;
        b = 100;
        cloudName = "cloud3";
        break;
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            single_color(cloudToEdit, r, g, b);

    if(!pviz.updatePointCloud(cloudToEdit, single_color, cloudName))
    {
        pviz.addPointCloud<pcl::PointXYZ>(cloudToEdit, single_color, cloudName);
        pviz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    }
}

Eigen::Matrix4f Pclwindow::getTransformMatrix(const int id)
{
    switch (id) {
        case 0:
            return *t_0_;
            break;
        case 1:
            return *t_1_;
            break;
        case 2:
            return *t_2_;
            break;
        case 3:
        default:
            return *t_3_;
            break;
    }
}

void Pclwindow::on_translationStepInput_textChanged(const QString &arg1)
{
    float newStep = -1;
    try
    {
        newStep = std::stof(arg1.toStdString());
        translation_step_ = newStep;
        std::cout << "new translation step: " << translation_step_ << std::endl;
    } catch (const std::exception& e) {

    }
}

void Pclwindow::on_rotationStepInput_textChanged(const QString &arg1)
{
    float newStep = -1;
    try
    {
        newStep = std::stof(arg1.toStdString());
        rotation_step_ = newStep;
        std::cout << "new rotation step: " << rotation_step_ << std::endl;
    } catch (const std::exception& e) {

    }
}
