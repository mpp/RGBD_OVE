#include <QApplication>

#include <opencv2/opencv.hpp>

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

int main(int argc, char** argv)
{
    /// STEP 1 ACQUISIZIONE DALLE 4 TELECAMERE
    // Mi servono coppie del tipo <URL, Cloud>

    /// STEP 2 ASSOCIAZIONE DELLE MATRICI DI TRASFORMAZIONE
    // Qui devo assegnare l'ID alle telecamere -> <URL, Cloud, ID>
    // I dati sono nel file YAML di configurazione

    /// STEP 3 ELABORAZIONE CLOUD E APPLICAZIONE DELLE MATRICI
    // Devo applicare le operazioni di segmentazione, ...  e trasformare le matrici

    /// STEP 4 INTERAZIONE CON L'UTENTE
    // Ok, viene fatta con l'applicazione QT
    QApplication app(argc, argv);
    Pclwindow w;
    w.show();

    app.exec();

    std::cout << "Saving the matrices and exiting..." << std::endl;

    /// STEP 5 SALVATAGGIO DELLE MATRICI DI TRASFORMAZIONE
    // Devo salvare i dati in un nuovo file YAML con le 4 coppie <URL, Matrice>
    cv::FileStorage fs("new_config.yml", cv::FileStorage::WRITE);

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
