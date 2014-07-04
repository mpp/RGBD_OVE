#include <QApplication>

#include <opencv2/opencv.hpp>
//#include <openni2/OpenNI.h>

#include <pcl\io\openni2_grabber.h>

#include <pcl/common/common_headers.h>

#include "simplecloudgrabber.h"

#include "pclwindow.h"

cv::Mat eigen4fToCV(const Eigen::Matrix4f &matrix)
{
    cv::Mat cvMat = cv::Mat::zeros(4,4,CV_32FC1);

    cvMat.at<float>(0,0) = matrix(0,0);
    cvMat.at<float>(0,1) = matrix(0,1);
    cvMat.at<float>(0,2) = matrix(0,2);
    cvMat.at<float>(0,3) = matrix(0,3);

    cvMat.at<float>(1,0) = matrix(1,0);
    cvMat.at<float>(1,1) = matrix(1,1);
    cvMat.at<float>(1,2) = matrix(1,2);
    cvMat.at<float>(1,3) = matrix(1,3);

    cvMat.at<float>(2,0) = matrix(2,0);
    cvMat.at<float>(2,1) = matrix(2,1);
    cvMat.at<float>(2,2) = matrix(2,2);
    cvMat.at<float>(2,3) = matrix(2,3);

    cvMat.at<float>(3,0) = matrix(3,0);
    cvMat.at<float>(3,1) = matrix(3,1);
    cvMat.at<float>(3,2) = matrix(3,2);
    cvMat.at<float>(3,3) = matrix(3,3);

    return cvMat;
}
Eigen::Matrix4f cvToEigen4f(const cv::Mat cvMat)
{
    if (cvMat.cols != 4 || cvMat.rows != 4)
    {
        return clinter::identity();
    }

    Eigen::Matrix4f matrix;

    matrix(0,0) = cvMat.at<float>(0,0);
    matrix(0,1) = cvMat.at<float>(0,1);
    matrix(0,2) = cvMat.at<float>(0,2);
    matrix(0,3) = cvMat.at<float>(0,3);

    matrix(1,0) = cvMat.at<float>(1,0);
    matrix(1,1) = cvMat.at<float>(1,1);
    matrix(1,2) = cvMat.at<float>(1,2);
    matrix(1,3) = cvMat.at<float>(1,3);

    matrix(2,0) = cvMat.at<float>(2,0);
    matrix(2,1) = cvMat.at<float>(2,1);
    matrix(2,2) = cvMat.at<float>(2,2);
    matrix(2,3) = cvMat.at<float>(2,3);

    matrix(3,0) = cvMat.at<float>(3,0);
    matrix(3,1) = cvMat.at<float>(3,1);
    matrix(3,2) = cvMat.at<float>(3,2);
    matrix(3,3) = cvMat.at<float>(3,3);

    return matrix;
}

void copyCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &source,
               pcl::PointCloud<pcl::PointXYZ>::Ptr &dest)
{
    dest->clear();
	for (int k = 0; k < source->points.size(); k++)
    {
		pcl::PointXYZ pt = source->points[k];
        dest->points.push_back(pt);
    }
    dest->width = dest->points.size();
    dest->height = 1;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr gcloud(new pcl::PointCloud<pcl::PointXYZ>());

void cloudcb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	std::cout << cloud->points.size() << std::endl;
	copyCloud(cloud, gcloud);
}

int main(int argc, char** argv)
{
    //////
    /// STEP 1 ACQUISIZIONE DALLE 4 TELECAMERE
    // Mi servono coppie del tipo <URI, Cloud>

	std::vector< std::pair<std::string, std::string> > connectedDevices;

	boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> 
		deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance ();
	
	/// DEBUG CODE TO GET CAMERAS URIs
	/*boost::shared_ptr<pcl::io::openni2::OpenNI2Device> 
		device = deviceManager->getDeviceByIndex(0);
	 
	std::cout << device->getUri() << std::endl;
	std::string tp = "test.yml";
    cv::FileStorage tmp(tp, cv::FileStorage::WRITE);
	tmp << "URICamera" << device->getUri();
    tmp.release();

	char c;
	cin >> c;
	return 0;*/

	if (deviceManager->getNumOfConnectedDevices () >= 4)
	{
		boost::shared_ptr<pcl::io::openni2::OpenNI2Device> 
			device = deviceManager->getDeviceByIndex(0);
		connectedDevices.push_back(std::make_pair<std::string, std::string>("#1", device->getUri()));
		device = deviceManager->getDeviceByIndex(1);
		connectedDevices.push_back(std::make_pair<std::string, std::string>("#2", device->getUri()));
		device = deviceManager->getDeviceByIndex(2);
		connectedDevices.push_back(std::make_pair<std::string, std::string>("#3", device->getUri()));
		device = deviceManager->getDeviceByIndex(3);
		connectedDevices.push_back(std::make_pair<std::string, std::string>("#4", device->getUri()));
	}

    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr >
        cloudsVector;

    // Take a cloud for each sensor
    for (int i = 0; i < connectedDevices.size(); i++)
    {
		std::string currentURI = connectedDevices[i].first;
		std::cout << currentURI << " - " << connectedDevices[i].second << std::endl;

		pcl::io::OpenNI2Grabber g(currentURI);
		
		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&cloudcb, _1);

		boost::signals2::connection cloud_connection = g.registerCallback (f);

		g.start();

        //SimpleCloudGrabber grabber(&g);
		//boost::thread grabberThread(grabber);

		while(gcloud->points.size() <= 0)
		{
		}
		g.stop();
		std::cout << "OK, get the cloud!" << std::endl;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud = grabber.getCloud();
		
		//grabber.stopGrabber();
		//grabberThread.join();

		pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud(new pcl::PointCloud<pcl::PointXYZ>());
		copyCloud(gcloud, tmpCloud);

		gcloud->clear();
		
        cloudsVector.push_back(tmpCloud);
    }

    //////
    /// STEP 2 ASSOCIAZIONE DELLE MATRICI DI TRASFORMAZIONE
    // Qui devo assegnare l'ID alle telecamere -> <URI, Cloud, ID>
    // I dati sono nel file YAML di configurazione

    std::cout << "Loading config data: " << std::endl;

    /// TODO: controllare che ci sia il file
	std::string confPath = "C:/Users/Administrator/Desktop/PCL TEST/RGBD_OVE/config/config.yml";
    cv::FileStorage config(confPath,
                           cv::FileStorage::READ);

	if (!config.isOpened())
	{
		std::cout << "Cannot open config file" << confPath << std::endl;
		return -1;
	}

    // Load URIs
    std::string
            URI0,
            URI1,
            URI2,
            URI3;

    config["URICamera0"] >> URI0;
    config["URICamera1"] >> URI1;
    config["URICamera2"] >> URI2;
    config["URICamera3"] >> URI3;

    std::cout << "URI camera 0: " << URI0 << std::endl;
    std::cout << "URI camera 1: " << URI1 << std::endl;
    std::cout << "URI camera 2: " << URI2 << std::endl;
    std::cout << "URI camera 3: " << URI3 << std::endl;

    // Load transform matrices
    cv::Mat
            t0 = cv::Mat::zeros(4,4,CV_32FC1),
            t1 = cv::Mat::zeros(4,4,CV_32FC1),
            t2 = cv::Mat::zeros(4,4,CV_32FC1),
            t3 = cv::Mat::zeros(4,4,CV_32FC1);

    config["transformMatrix0"] >> t0;
    config["transformMatrix1"] >> t1;
    config["transformMatrix2"] >> t2;
    config["transformMatrix3"] >> t3;

    std::cout << "Transform matrix 0: " << std::endl << t0 << std::endl;
    std::cout << "Transform matrix 1: " << std::endl << t1 << std::endl;
    std::cout << "Transform matrix 2: " << std::endl << t2 << std::endl;
    std::cout << "Transform matrix 3: " << std::endl << t3 << std::endl;

    /// TODO: Match URIs, transformations and clouds
    // Riordinare il vettore di cloud
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr >  sortedClouds;
	for (int k = 0; k < connectedDevices.size(); k++)
	{
		if (URI0.compare(connectedDevices[k].second) == 0)
		{
			sortedClouds.push_back(cloudsVector[k]);
			break;
		}
	}
	for (int k = 0; k < connectedDevices.size(); k++)
	{
		if (URI1.compare(connectedDevices[k].second) == 0)
		{
			sortedClouds.push_back(cloudsVector[k]);
			break;
		}
	}
	for (int k = 0; k < connectedDevices.size(); k++)
	{
		if (URI2.compare(connectedDevices[k].second) == 0)
		{
			sortedClouds.push_back(cloudsVector[k]);
			break;
		}
	}
	for (int k = 0; k < connectedDevices.size(); k++)
	{
		if (URI3.compare(connectedDevices[k].second) == 0)
		{
			sortedClouds.push_back(cloudsVector[k]);
			break;
		}
	}

	std::cout << "Are there 4 clouds? " << sortedClouds.size() << std::endl;

	if (sortedClouds.size() != 4)
		return 0;

    //////
    /// STEP 3 ELABORAZIONE CLOUD E APPLICAZIONE DELLE MATRICI
    // Devo applicare le operazioni di segmentazione, ...  e trasformare le matrici

    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > elaboratedCloudsVector;
    for (int k = 0; k < sortedClouds.size(); k++)
    {
		std::cout << "Loading cloud " << k << "...";
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp;
        clinter::elaborateCloud(sortedClouds[k], tmp);

        Eigen::Matrix4f transform;

        switch (k) {
        case 0:
            transform = cvToEigen4f(t0);
            break;
        case 1:
            transform = cvToEigen4f(t1);
            break;
        case 2:
            transform = cvToEigen4f(t2);
            break;
        case 3:
        default:
            transform = cvToEigen4f(t3);
            break;
        }

        pcl::transformPointCloud(*tmp, *tmp, transform);

        elaboratedCloudsVector.push_back(tmp);

		std::cout << "ok" << std::endl;
    }

    //////
    /// STEP 4 INTERAZIONE CON L'UTENTE
    // Viene fatta con l'applicazione QT
    QApplication app(argc, argv);
    Pclwindow w;
    if (elaboratedCloudsVector.size() == 4)
    {
        w.loadCloud0(elaboratedCloudsVector[0]);
        w.loadCloud1(elaboratedCloudsVector[1]);
        w.loadCloud2(elaboratedCloudsVector[2]);
        w.loadCloud3(elaboratedCloudsVector[3]);
    }
    w.show();

    app.exec();

    std::cout << "Saving the matrices and exiting..." << std::endl;

    //////
    /// STEP 5 SALVATAGGIO DELLE MATRICI DI TRASFORMAZIONE
    // Devo salvare i dati in un nuovo file YAML con le 4 coppie <URI, Matrice>

    /// TODO: Controllare che la destinazione sia scrivibile
	std::string outPath = "new_config.yml";
    cv::FileStorage fs(outPath, cv::FileStorage::WRITE);

	if (!fs.isOpened())
	{
		std::cerr << "Cannot open output file: " << outPath << std::endl;
		return -1;
	}

    fs << "URICamera0" << URI0;
    fs << "URICamera1" << URI1;
    fs << "URICamera2" << URI2;
    fs << "URICamera3" << URI3;

    cv::Mat mat = eigen4fToCV(w.getTransformMatrix(0));
    fs << "transformMatrix0" << mat;
    mat = eigen4fToCV(w.getTransformMatrix(1));
    fs << "transformMatrix1" << mat;
    mat = eigen4fToCV(w.getTransformMatrix(2));
    fs << "transformMatrix2" << mat;
    mat = eigen4fToCV(w.getTransformMatrix(3));
    fs << "transformMatrix3" << mat;

    fs.release();

    return EXIT_SUCCESS;
}
