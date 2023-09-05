#include <cassert>
#include <filesystem>

#define vtkRenderingContext2D_AUTOINIT 1(vtkRenderingContextOpenGL2)
#define vtkRenderingCore_AUTOINIT 3(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingOpenGL2)
#define vtkRenderingOpenGL2_AUTOINIT 1(vtkRenderingGL2PSOpenGL2)

#include <vtkAxis.h>
#include <vtkBrush.h>
#include <vtkBMPWriter.h>
#include <vtkChartLegend.h>
#include <vtkChartMatrix.h>
#include <vtkChartXY.h>
#include <vtkContextScene.h>
#include <vtkContextView.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkImageWriter.h>
#include <vtkJPEGWriter.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPen.h>
#include <vtkPlot.h>
#include <vtkPlotArea.h>
#include <vtkPlotFunctionalBag.h>
#include <vtkPlotPoints.h>
#include <vtkPNGWriter.h>
#include <vtkPNMWriter.h>
#include <vtkPostScriptWriter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTable.h>
#include <vtkTextProperty.h>
#include <vtkTIFFWriter.h>
#include <vtkWindowToImageFilter.h>

#include "ballistic_plot.h"

void WriteImage(std::filesystem::path path, vtkRenderWindow * renWin, bool rgba)
{
    if (path.empty())
    {
        std::cerr << "No path provided." << std::endl;
        return;
    }
    else
    {
        std::string ext = path.extension().string();
        if (ext.empty())
        {
            ext = ".png";
            path = path / ".png";
        }
        std::locale loc;
        std::transform(ext.begin(), ext.end(), ext.begin(),
           [=](char const & c) { return std::tolower(c, loc); });
        auto writer = vtkSmartPointer<vtkImageWriter>::New();
        if (ext == ".bmp")
        {
            writer = vtkSmartPointer<vtkBMPWriter>::New();
        }
        else if (ext == ".jpg")
        {
            writer = vtkSmartPointer<vtkJPEGWriter>::New();
        }
        else if (ext == ".pnm")
        {
            writer = vtkSmartPointer<vtkPNMWriter>::New();
        }
        else if (ext == ".ps")
        {
            rgba = false;
            writer = vtkSmartPointer<vtkPostScriptWriter>::New();
        }
        else if (ext == ".tiff")
        {
            writer = vtkSmartPointer<vtkTIFFWriter>::New();
        }
        else
        {
            writer = vtkSmartPointer<vtkPNGWriter>::New();
        }

        vtkNew<vtkWindowToImageFilter> window_to_image_filter;
        window_to_image_filter->SetInput(renWin);
        window_to_image_filter->SetScale(1); // image quality
        if (rgba)
        {
            window_to_image_filter->SetInputBufferTypeToRGBA();
        }
        else
        {
            window_to_image_filter->SetInputBufferTypeToRGB();
        }
        // Read from the front buffer.
        window_to_image_filter->ReadFrontBufferOff();
        window_to_image_filter->Update();

        writer->SetFileName(path.string().c_str());
        writer->SetInputConnection(window_to_image_filter->GetOutputPort());
        writer->Write();
    }
}

void plot_simulation(
    const Eigen::VectorXd & tHist, 
    const Eigen::MatrixXd & xHist, 
    const Eigen::MatrixXd & muHist, 
    const Eigen::MatrixXd & sigmaHist, 
    const Eigen::MatrixXd & hHist, 
    const Eigen::MatrixXd & yHist)
{

    int nsteps  = xHist.cols();
    int nx      = 3;
    int ny      = 1;
    assert(xHist.rows() == nx);
    assert(muHist.rows() == nx);
    assert(hHist.rows() == ny);
    assert(hHist.rows() == ny);

    assert(tHist.size() == nsteps);
    assert(muHist.cols() == nsteps);
    assert(sigmaHist.cols() == nsteps);
    assert(hHist.cols() == nsteps);
    assert(yHist.cols() == nsteps);

    // Sigma for the 99.7% confidence region
    double sigma  = 3;

    // Font size, colours, and linewidth
    // -----------------------------------------------
    auto title_fontsize   = 36;
    auto axis_fontsize    = 24;
    auto label_fontsize   = 18;
    auto legend_fontsize  = 18;
    auto linewidth        = 1.0;
    auto title_fontcolour  = "black";
    auto axis_fontcolour   = "black";
    auto label_fontcolour  = "black";

    // -----------------------------------------------
    // -----------------------------------------------
    // Instantiate the render stuff
    // -----------------------------------------------
    // -----------------------------------------------

    vtkNew<vtkContextView> view;
    vtkRenderWindow *renWin = view->GetRenderWindow();
    renWin->SetSize(1024, 768);

    vtkNew<vtkNamedColors> colors;

    // -----------------------------------------------
    // -----------------------------------------------
    // Setup the chart matrix (similar to sub plots
    //   in matlab)
    // -----------------------------------------------
    // -----------------------------------------------

    vtkNew<vtkChartMatrix> matrix;
    view->GetScene()->AddItem(matrix);
    matrix->SetSize(vtkVector2i(2, 2));
    matrix->SetGutter(vtkVector2f(100.0, 100.0));
    matrix->SetBorders(100, 50, 50, 50);

    // -----------------------------------------------
    // -----------------------------------------------
    // Create the charts.
    // -----------------------------------------------
    // -----------------------------------------------

    // Top left
    // -----------------------------------------------
    vtkChart *topLeftChart = matrix->GetChart(vtkVector2i(0, 1));

    auto xAxis = topLeftChart->GetAxis(vtkAxis::BOTTOM);
    xAxis->GetGridPen()->SetColor(colors->GetColor4ub("LightCyan"));
    xAxis->SetTitle("Time [s]");
    xAxis->GetTitleProperties()->SetFontSize(axis_fontsize);
    xAxis->GetTitleProperties()->SetColor(colors->GetColor3d(axis_fontcolour).GetData());
    xAxis->GetLabelProperties()->SetFontSize(label_fontsize);
    xAxis->GetLabelProperties()->SetColor(colors->GetColor3d(label_fontcolour).GetData());

    auto yAxis = topLeftChart->GetAxis(vtkAxis::LEFT);
    yAxis->GetGridPen()->SetColor(colors->GetColor4ub("LightCyan"));
    yAxis->SetTitle("Height [m]");
    yAxis->GetTitleProperties()->SetFontSize(axis_fontsize);
    yAxis->GetTitleProperties()->SetColor(colors->GetColor3d(axis_fontcolour).GetData());
    yAxis->GetLabelProperties()->SetFontSize(label_fontsize);
    yAxis->GetLabelProperties()->SetColor(colors->GetColor3d(label_fontcolour).GetData());


    topLeftChart->GetBackgroundBrush()->SetColorF(
      colors->GetColor3d("SlateGray").GetData());
    topLeftChart->GetBackgroundBrush()->SetOpacityF(0.4);
    topLeftChart->SetTitle("Altitude");
    topLeftChart->GetTitleProperties()->SetFontSize(title_fontsize);
    topLeftChart->GetTitleProperties()->SetColor(colors->GetColor3d(title_fontcolour).GetData());


    // Top right
    // -----------------------------------------------
    vtkChart *topRightChart = matrix->GetChart(vtkVector2i(1, 1));

    // Background
    topRightChart->GetBackgroundBrush()->SetColorF(
      colors->GetColor3d("SlateGray").GetData());
    topRightChart->GetBackgroundBrush()->SetOpacityF(0.4);

    // Title
    topRightChart->SetTitle("Range");
    topRightChart->GetTitleProperties()->SetFontSize(title_fontsize);
    topRightChart->GetTitleProperties()->SetColor(colors->GetColor3d(title_fontcolour).GetData());

    // X axis
    xAxis = topRightChart->GetAxis(vtkAxis::BOTTOM);
    xAxis->GetGridPen()->SetColor(colors->GetColor4ub("LightCyan"));
    xAxis->SetTitle("Time [s]");
    xAxis->GetTitleProperties()->SetFontSize(axis_fontsize);
    xAxis->GetTitleProperties()->SetColor(colors->GetColor3d(axis_fontcolour).GetData());
    xAxis->GetLabelProperties()->SetFontSize(label_fontsize);
    xAxis->GetLabelProperties()->SetColor(colors->GetColor3d(label_fontcolour).GetData());

    // Y axis
    yAxis = topRightChart->GetAxis(vtkAxis::LEFT);
    yAxis->GetGridPen()->SetColor(colors->GetColor4ub("LightCyan"));
    yAxis->SetTitle("Range [m]");
    yAxis->GetTitleProperties()->SetFontSize(axis_fontsize);
    yAxis->GetTitleProperties()->SetColor(colors->GetColor3d(axis_fontcolour).GetData());
    yAxis->GetLabelProperties()->SetFontSize(label_fontsize);
    yAxis->GetLabelProperties()->SetColor(colors->GetColor3d(label_fontcolour).GetData());


    // Bottom left
    // -----------------------------------------------
    vtkChart *bottomLeftChart = matrix->GetChart(vtkVector2i(0, 0));

    // Background
    bottomLeftChart->GetBackgroundBrush()->SetColorF(
      colors->GetColor3d("SlateGray").GetData());
    bottomLeftChart->GetBackgroundBrush()->SetOpacityF(0.4);
    
    // Title
    bottomLeftChart->SetTitle("Velocity");
    bottomLeftChart->GetTitleProperties()->SetFontSize(title_fontsize);
    bottomLeftChart->GetTitleProperties()->SetColor(colors->GetColor3d(title_fontcolour).GetData());

    // X axis
    xAxis = bottomLeftChart->GetAxis(vtkAxis::BOTTOM);
    xAxis->GetGridPen()->SetColor(colors->GetColor4ub("LightCyan"));
    xAxis->SetTitle("Time [s]");
    xAxis->GetTitleProperties()->SetFontSize(axis_fontsize);
    xAxis->GetTitleProperties()->SetColor(colors->GetColor3d(axis_fontcolour).GetData());
    xAxis->GetLabelProperties()->SetFontSize(label_fontsize);
    xAxis->GetLabelProperties()->SetColor(colors->GetColor3d(label_fontcolour).GetData());

    // Y axis
    yAxis = bottomLeftChart->GetAxis(vtkAxis::LEFT);
    yAxis->GetGridPen()->SetColor(colors->GetColor4ub("LightCyan"));
    yAxis->SetTitle("Velocity [m/s]");
    yAxis->GetTitleProperties()->SetFontSize(axis_fontsize);
    yAxis->GetTitleProperties()->SetColor(colors->GetColor3d(axis_fontcolour).GetData());
    yAxis->GetLabelProperties()->SetFontSize(label_fontsize);
    yAxis->GetLabelProperties()->SetColor(colors->GetColor3d(label_fontcolour).GetData());

    // Bottom right
    // -----------------------------------------------

    vtkChart *bottomRightChart = matrix->GetChart(vtkVector2i(1, 0));

    // Background
    bottomRightChart->GetBackgroundBrush()->SetColorF(
    colors->GetColor3d("SlateGray").GetData());
    bottomRightChart->GetBackgroundBrush()->SetOpacityF(0.4);

    // Title
    bottomRightChart->SetTitle("Ballistic Coefficient");
    bottomRightChart->GetTitleProperties()->SetFontSize(title_fontsize);
    bottomRightChart->GetTitleProperties()->SetColor(colors->GetColor3d(title_fontcolour).GetData());

    // X axis
    xAxis = bottomRightChart->GetAxis(vtkAxis::BOTTOM);
    xAxis->GetGridPen()->SetColor(colors->GetColor4ub("LightCyan"));
    xAxis->SetTitle("Time [s]");
    xAxis->GetTitleProperties()->SetFontSize(axis_fontsize);
    xAxis->GetTitleProperties()->SetColor(colors->GetColor3d(axis_fontcolour).GetData());
    xAxis->GetLabelProperties()->SetFontSize(label_fontsize);
    xAxis->GetLabelProperties()->SetColor(colors->GetColor3d(label_fontcolour).GetData());

    // Y axis
    yAxis = bottomRightChart->GetAxis(vtkAxis::LEFT);
    yAxis->GetGridPen()->SetColor(colors->GetColor4ub("LightCyan"));
    yAxis->SetTitle("Ballistic Coefficient [m²/kg]");
    yAxis->GetTitleProperties()->SetFontSize(axis_fontsize);
    yAxis->GetTitleProperties()->SetColor(colors->GetColor3d(axis_fontcolour).GetData());
    yAxis->GetLabelProperties()->SetFontSize(label_fontsize);
    yAxis->GetLabelProperties()->SetColor(colors->GetColor3d(label_fontcolour).GetData());


    /*
     *          __.--,
     *     ,--'~-.,;=/
     *    {  /,”” ‘ |
     *     `|\_ (@\(@\_
     *      \(  _____(./
     *      | `-~--~-‘\
     *      | /   /  , \
     *      | |   (\  \  \
     *     _\_|.-‘` `’| ,-= )
     *    (_  ,\ ____  \  ,/
     *      \  |--===--(,,/
     *       ```========-/
     *         \_______ /
     *  SOMEBODY TOUCHA MY SPAGHET!
     */ 

    // -----------------------------------------------
    // -----------------------------------------------
    // Create data table
    // -----------------------------------------------
    // -----------------------------------------------
    vtkNew<vtkTable> table;

    vtkNew<vtkFloatArray> 
        arr_t,
        arr_height_true,
        arr_height_est,
        arr_velocity_true,
        arr_velocity_est,
        arr_bcoeff_true,
        arr_bcoeff_est,
        arr_range_true,
        arr_range_meas,
        arr_mu1_plus_sigma1,
        arr_mu1_minus_sigma1,
        arr_mu2_plus_sigma2,
        arr_mu2_minus_sigma2,
        arr_mu3_plus_sigma3,
        arr_mu3_minus_sigma3;

    //  Initialise the table
    // -----------------------------------------------
    enum
    {
        // Means
        TABLE_TIME,
        TABLE_HEIGHT_TRUE,
        TABLE_HEIGHT_EST,
        TABLE_VELOCITY_TRUE,
        TABLE_VELOCITY_EST,
        TABLE_BCOEFF_TRUE,
        TABLE_BCOEFF_EST,
        TABLE_RANGE_TRUE,
        TABLE_RANGE_MEAS,

        // Confidence region
        TABLE_MU1_PLUS_SIGMA1,
        TABLE_MU1_MINUS_SIGMA1,
        TABLE_MU2_PLUS_SIGMA2,
        TABLE_MU2_MINUS_SIGMA2,
        TABLE_MU3_PLUS_SIGMA3,
        TABLE_MU3_MINUS_SIGMA3,
    };

    #define KEY(NAME) "KEY_" #NAME

    // Make columns of the table
    arr_t->SetName(KEY(TABLE_TIME)); table->AddColumn(arr_t);
    
    arr_height_true->SetName(KEY(TABLE_HEIGHT_TRUE)); table->AddColumn(arr_height_true);
    arr_height_est->SetName(KEY(TABLE_HEIGHT_EST)); table->AddColumn(arr_height_est);
    
    arr_velocity_true->SetName(KEY(TABLE_VELOCITY_TRUE)); table->AddColumn(arr_velocity_true);
    arr_velocity_est->SetName(KEY(TABLE_VELOCITY_EST)); table->AddColumn(arr_velocity_est);
    
    arr_bcoeff_true->SetName(KEY(TABLE_BCOEFF_TRUE)); table->AddColumn(arr_bcoeff_true);
    arr_bcoeff_est->SetName(KEY(TABLE_BCOEFF_EST)); table->AddColumn(arr_bcoeff_est);
    
    arr_range_true->SetName(KEY(TABLE_RANGE_TRUE)); table->AddColumn(arr_range_true);
    arr_range_meas->SetName(KEY(TABLE_RANGE_MEAS)); table->AddColumn(arr_range_meas);
    
    arr_mu1_plus_sigma1->SetName(KEY(TABLE_MU1_PLUS_SIGMA1)); table->AddColumn(arr_mu1_plus_sigma1);
    arr_mu1_minus_sigma1->SetName(KEY(TABLE_MU1_MINUS_SIGMA1)); table->AddColumn(arr_mu1_minus_sigma1);

    arr_mu2_plus_sigma2->SetName(KEY(TABLE_MU2_PLUS_SIGMA2)); table->AddColumn(arr_mu2_plus_sigma2);
    arr_mu2_minus_sigma2->SetName(KEY(TABLE_MU2_MINUS_SIGMA2)); table->AddColumn(arr_mu2_minus_sigma2);

    arr_mu3_plus_sigma3->SetName(KEY(TABLE_MU3_PLUS_SIGMA3)); table->AddColumn(arr_mu3_plus_sigma3);
    arr_mu3_minus_sigma3->SetName(KEY(TABLE_MU3_MINUS_SIGMA3)); table->AddColumn(arr_mu3_minus_sigma3);


    //  Fill in the table with data from the simulation
    // -----------------------------------------------
    table->SetNumberOfRows(nsteps);

    for (int i = 0; i < nsteps; ++i)
    {
        table->SetValue(i,                  TABLE_TIME,  tHist(i));
        table->SetValue(i,           TABLE_HEIGHT_TRUE,  xHist(0,i));
        table->SetValue(i,         TABLE_VELOCITY_TRUE,  xHist(1,i));
        table->SetValue(i,           TABLE_BCOEFF_TRUE,  xHist(2,i));

        table->SetValue(i,            TABLE_HEIGHT_EST,  muHist(0,i));
        table->SetValue(i,          TABLE_VELOCITY_EST,  muHist(1,i));
        table->SetValue(i,            TABLE_BCOEFF_EST,  muHist(2,i));

        table->SetValue(i,            TABLE_RANGE_TRUE,  hHist(0,i));
        table->SetValue(i,            TABLE_RANGE_MEAS,  yHist(0,i));

        // 
        table->SetValue(i,       TABLE_MU1_PLUS_SIGMA1,  muHist(0,i) + sigma*sigmaHist(0,i));
        table->SetValue(i,      TABLE_MU1_MINUS_SIGMA1,  muHist(0,i) - sigma*sigmaHist(0,i));
        // 
        table->SetValue(i,       TABLE_MU2_PLUS_SIGMA2,  muHist(1,i) + sigma*sigmaHist(1,i));
        table->SetValue(i,      TABLE_MU2_MINUS_SIGMA2,  muHist(1,i) - sigma*sigmaHist(1,i));
        // 
        table->SetValue(i,       TABLE_MU3_PLUS_SIGMA3,  muHist(2,i) + sigma*sigmaHist(2,i));
        table->SetValue(i,      TABLE_MU3_MINUS_SIGMA3,  muHist(2,i) - sigma*sigmaHist(2,i));
    }

    // -----------------------------------------------
    // -----------------------------------------------
    // Plot things
    // -----------------------------------------------
    // -----------------------------------------------
    vtkColor3d color3d = colors->GetColor3d("tomato");
    vtkPlot* line;
    vtkPlotArea* area;


    // Top left plot
    // -----------------------------------------------
    area = dynamic_cast<vtkPlotArea *>(topLeftChart->AddPlot(vtkChart::AREA));
    area->SetInputData(table);
    area->SetInputArray(0, KEY(TABLE_TIME));
    area->SetInputArray(1, KEY(TABLE_MU1_PLUS_SIGMA1));
    area->SetInputArray(2, KEY(TABLE_MU1_MINUS_SIGMA1));
    area->GetBrush()->SetColorF(color3d.GetRed(), color3d.GetGreen(),
                                color3d.GetBlue(), .3);
    area->SetLabel("99.7% confidence region");

    line = topLeftChart->AddPlot(vtkChart::LINE);
    line->SetInputData(table, 0, TABLE_HEIGHT_TRUE);
    line->SetColor(0, 0, 255, 255);
    line->SetWidth(linewidth);
    line->SetLabel("True");

    line = topLeftChart->AddPlot(vtkChart::LINE);
    line->SetInputData(table, 0, TABLE_HEIGHT_EST);
    line->SetColor(255, 125, 0, 255);
    line->SetWidth(linewidth);
    line->SetLabel("Estimated");

    // Show legend
    topLeftChart->SetShowLegend(true);
    topLeftChart->GetLegend()->SetHorizontalAlignment(vtkChartLegend::RIGHT);
    topLeftChart->GetLegend()->SetVerticalAlignment(vtkChartLegend::TOP);
    topLeftChart->GetLegend()->SetLabelSize(legend_fontsize);
    
    // Top right plot
    // -----------------------------------------------
    line = topRightChart->AddPlot(vtkChart::LINE);
    line->SetInputData(table, 0, TABLE_RANGE_TRUE);
    line->SetColor(0, 0, 255, 255);
    line->SetWidth(linewidth);
    line->SetLabel("True");

    line = topRightChart->AddPlot(vtkChart::LINE);
    line->SetInputData(table, 0, TABLE_RANGE_MEAS);
    line->SetColor(255, 125, 0, 255);
    line->SetWidth(linewidth);
    line->SetLabel("Measured");

    // Show legend
    topRightChart->SetShowLegend(true);
    topRightChart->GetLegend()->SetHorizontalAlignment(vtkChartLegend::RIGHT);
    topRightChart->GetLegend()->SetVerticalAlignment(vtkChartLegend::TOP);
    topRightChart->GetLegend()->SetLabelSize(legend_fontsize);

    // Bottom left plot
    // -----------------------------------------------
    area = dynamic_cast<vtkPlotArea *>(bottomLeftChart->AddPlot(vtkChart::AREA));
    area->SetInputData(table);
    area->SetInputArray(0, KEY(TABLE_TIME));
    area->SetInputArray(1, KEY(TABLE_MU2_PLUS_SIGMA2));
    area->SetInputArray(2, KEY(TABLE_MU2_MINUS_SIGMA2));
    area->GetBrush()->SetColorF(color3d.GetRed(), color3d.GetGreen(),
                                color3d.GetBlue(), .3);
    area->SetLabel("99.7% confidence region");

    line = bottomLeftChart->AddPlot(vtkChart::LINE);
    line->SetInputData(table, 0, TABLE_VELOCITY_TRUE);
    line->SetColor(0, 0, 255, 255);
    line->SetWidth(linewidth);
    line->SetLabel("True");

    line = bottomLeftChart->AddPlot(vtkChart::LINE);
    line->SetInputData(table, 0, TABLE_VELOCITY_EST);
    line->SetColor(255, 125, 0, 255);
    line->SetWidth(linewidth);    
    line->SetLabel("Estimated");

    // Show legend
    bottomLeftChart->SetShowLegend(true);
    bottomLeftChart->GetLegend()->SetHorizontalAlignment(vtkChartLegend::RIGHT);
    bottomLeftChart->GetLegend()->SetVerticalAlignment(vtkChartLegend::BOTTOM);
    bottomLeftChart->GetLegend()->SetLabelSize(legend_fontsize);

    // Bottom right plot
    // -----------------------------------------------

    // Confidence region
    area = dynamic_cast<vtkPlotArea *>(bottomRightChart->AddPlot(vtkChart::AREA));
    area->SetInputData(table);
    area->SetInputArray(0, KEY(TABLE_TIME));
    area->SetInputArray(1, KEY(TABLE_MU3_PLUS_SIGMA3));
    area->SetInputArray(2, KEY(TABLE_MU3_MINUS_SIGMA3));
    area->GetBrush()->SetColorF(color3d.GetRed(), color3d.GetGreen(),
                                color3d.GetBlue(), .3);
    area->SetLabel("99.7% confidence region");

    // True state
    line = bottomRightChart->AddPlot(vtkChart::LINE);
    line->SetInputData(table, 0, TABLE_BCOEFF_TRUE);
    line->SetColor(0, 0, 255, 255);
    line->SetWidth(linewidth);
    line->SetLabel("True");

    // Estimated state
    line = bottomRightChart->AddPlot(vtkChart::LINE);
    line->SetInputData(table, 0, TABLE_BCOEFF_EST);
    line->SetColor(255, 125, 0, 255);
    line->SetWidth(linewidth);
    line->SetLabel("Estimated");

    // Show legend
    bottomRightChart->SetShowLegend(true);
    bottomRightChart->GetLegend()->SetHorizontalAlignment(vtkChartLegend::RIGHT);
    bottomRightChart->GetLegend()->SetVerticalAlignment(vtkChartLegend::BOTTOM);
    bottomRightChart->GetLegend()->SetLabelSize(legend_fontsize);

    /*
     *          __.--,
     *     ,--'~-.,;=/
     *    {  /,”” ‘ |
     *     `|\_ (@\(@\_
     *      \(  _____(./
     *      | `-~--~-‘\
     *      | /   /  , \
     *      | |   (\  \  \
     *     _\_|.-‘` `’| ,-= )
     *    (_  ,\ ____  \  ,/
     *      \  |--===--(,,/
     *       ```========-/
     *         \_______ /
     *  SOMEBODY TOUCHA MY SPAGHET!
     */  

    #undef KEY

    // Do the render
    // -----------------------------------------------
    view->GetRenderer()->SetBackground(colors->GetColor3d("White").GetData());
    renWin->SetMultiSamples(0);
    renWin->Render();
    renWin->SetWindowName("Ballistic IEKF");

    WriteImage("../data/Ballistic IEKF.png", renWin, false);

    vtkRenderWindowInteractor *iRen = view->GetInteractor();
    iRen->Initialize();
    iRen->Start();
}
