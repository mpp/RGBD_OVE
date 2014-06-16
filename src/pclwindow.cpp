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
}

Pclwindow::~Pclwindow()
{
    delete ui;
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
    clinter::x_t_inc_callback(cloudToEdit(), translation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_yTIncButton_clicked()
{
    clinter::y_t_inc_callback(cloudToEdit(), translation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_zTIncButton_clicked()
{
    clinter::z_t_inc_callback(cloudToEdit(), translation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_xTDecButton_clicked()
{
    clinter::x_t_dec_callback(cloudToEdit(), translation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_yTDecButton_clicked()
{
    clinter::y_t_dec_callback(cloudToEdit(), translation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_zTDecButton_clicked()
{
    clinter::z_t_dec_callback(cloudToEdit(), translation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_xRIncButton_clicked()
{
    clinter::x_r_inc_callback(cloudToEdit(), rotation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_xRDecButton_clicked()
{
    clinter::x_r_dec_callback(cloudToEdit(), rotation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_yRIncButton_clicked()
{
    clinter::y_r_inc_callback(cloudToEdit(), rotation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_yRDecButton_clicked()
{
    clinter::y_r_dec_callback(cloudToEdit(), rotation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_zRIncButton_clicked()
{
    clinter::z_r_inc_callback(cloudToEdit(), rotation_step_);
    updateCloud(cloudToEdit());
}

void Pclwindow::on_zRDecButton_clicked()
{
    clinter::z_r_dec_callback(cloudToEdit(), rotation_step_);
    updateCloud(cloudToEdit());
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
