#include <QApplication>
#include <QMainWindow>

#include "pclviewer.h"
//#include "vtkoutputwindow.h"  //加入
int main(int argc, char* argv[]) {
    // vtkOutputWindow::SetGlobalWarningDisplay(0);  //加入
    // vtkOutputWindow* vw = vtkOutputWindow::GetInstance();
    // vw->Delete();
    QApplication a(argc, argv);
    PCLViewer w;
    w.show();

    return a.exec();
}
