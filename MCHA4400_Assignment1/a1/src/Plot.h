#ifndef PLOT_H
#define PLOT_H

#include <string>
#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <vtkActor.h>
#include <vtkActor2D.h>
#include <vtkCellArray.h>
#include <vtkColor.h>
#include <vtkContourFilter.h>
#include <vtkCubeAxesActor.h>
#include <vtkDataSetMapper.h>
#include <vtkImageData.h>
#include <vtkImageMapper.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPyramid.h>
#include <vtkQuadric.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSampleFunction.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkUnstructuredGrid.h>

#include "Camera.h"
#include "Gaussian.hpp"
#include "StateSLAM.h"


// -------------------------------------------------------
// Bounds
// -------------------------------------------------------
struct Bounds
{
    Bounds();
    void getVTKBounds(double * bounds) const;
    void setExtremity(Bounds & extremity) const;
    void calculateMaxMinSigmaPoints(const Gaussian<double> & positionDensity, const double sigma);

    double xmin, xmax;
    double ymin, ymax;
    double zmin, zmax;
};

// -------------------------------------------------------
// QuadricPlot
// -------------------------------------------------------
struct QuadricPlot
{
    QuadricPlot();
    void update(const Gaussian<double> & positionDensity);
    vtkActor * getActor() const;
    Bounds bounds;
    vtkSmartPointer<vtkActor>            contourActor;
    vtkSmartPointer<vtkContourFilter>    contours;
    vtkSmartPointer<vtkPolyDataMapper>   contourMapper;
    vtkSmartPointer<vtkQuadric>          quadric;
    vtkSmartPointer<vtkSampleFunction>   sample;
    const double value;
    bool isInit;
};

// -------------------------------------------------------
// FrustumPlot
// -------------------------------------------------------
struct FrustumPlot
{
    explicit FrustumPlot(const Camera & camera);
    void update(const Eigen::Vector3d & rCNn, const Eigen::Vector3d & Thetanc);
    vtkActor * getActor() const;

    vtkSmartPointer<vtkActor> pyramidActor;
    vtkSmartPointer<vtkCellArray> cells;
    vtkSmartPointer<vtkDataSetMapper> mapper;
    vtkSmartPointer<vtkPoints> pyramidPts;
    vtkSmartPointer<vtkPyramid> pyramid;
    vtkSmartPointer<vtkUnstructuredGrid> ug;
    Eigen::MatrixXd rPCc;
    Eigen::MatrixXd rPNn;
    bool isInit;
};

// -------------------------------------------------------
// AxisPlot
// -------------------------------------------------------
struct AxisPlot
{
    AxisPlot();
    void init(vtkCamera *cam);
    void update(const Bounds & bounds);
    vtkActor * getActor() const;

    vtkColor3d axis1Color;
    vtkColor3d axis2Color;
    vtkColor3d axis3Color;
    vtkSmartPointer<vtkCubeAxesActor> cubeAxesActor;
    bool isInit;
};

// -------------------------------------------------------
// BasisPlot
// -------------------------------------------------------
struct BasisPlot
{
    BasisPlot();
    void update(const Eigen::Vector3d & rCNn, const Eigen::Vector3d & Thetanc);
    vtkActor * getActor() const;

    vtkSmartPointer<vtkActor>            basisActor;
    vtkSmartPointer<vtkCellArray>        lines;
    vtkSmartPointer<vtkLine>             line0, line1, line2;
    vtkSmartPointer<vtkPoints>           basisPts;
    vtkSmartPointer<vtkPolyData>         linesPolyData;
    vtkSmartPointer<vtkPolyDataMapper>   basisMapper;
    vtkSmartPointer<vtkUnsignedCharArray> colorSet;
    Eigen::MatrixXd rPNn;
    Eigen::VectorXd rCNn;
    bool isInit;
};

// -------------------------------------------------------
// ImagePlot
// -------------------------------------------------------
struct ImagePlot
{
    ImagePlot();
    void init(double rendererWidth, double rendererHeight);
    void update(const cv::Mat & view);
    vtkActor2D * getActor() const;

    vtkSmartPointer<vtkImageData> viewVTK;
    vtkSmartPointer<vtkActor2D> imageActor2d;
    vtkSmartPointer<vtkImageMapper> imageMapper;
    cv::Mat cvVTKBuffer;
    double width, height;
    bool isInit;
};


struct Plot
{
public:
    Plot(const StateSLAM & state, const Camera & camera);
    void render();
    void start() const;
    void setState(const StateSLAM & state);
    cv::Mat getFrame() const;

private:
    std::unique_ptr<StateSLAM> pState;
    const Camera & camera;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderer>     threeDimRenderer;
    vtkSmartPointer<vtkRenderer>     imageRenderer;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;
    QuadricPlot qpCamera;
    std::vector<QuadricPlot> qpLandmarks;
    FrustumPlot fp;
    AxisPlot ap;
    BasisPlot bp;
    ImagePlot ip;
};

#endif
