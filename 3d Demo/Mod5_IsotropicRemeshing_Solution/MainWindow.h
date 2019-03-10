#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtOpenGL/qgl.h>
#include <CGAL/Qt/DemosMainWindow.h>

class QDragEnterEvent;
class QDropEvent;
class Scene;
class Viewer;
namespace Ui {
	class MainWindow;
}


class MainWindow : 
	public CGAL::Qt::DemosMainWindow
{
	Q_OBJECT
public:
	MainWindow(QWidget* parent = 0);
	~MainWindow();

        public slots:
            void updateViewerBBox();
            void open(QString filename);
            void setAddKeyFrameKeyboardModifiers(Qt::KeyboardModifiers);

        protected slots:
            // settings
            void quit();
            void readSettings();
            void writeSettings();

            // drag & drop
            void dropEvent(QDropEvent *event);
            void closeEvent(QCloseEvent *event);
            void dragEnterEvent(QDragEnterEvent *event);

            // file menu
            void on_actionLoadPolyhedron_triggered();

            // edit menu
            void on_actionSave_snapshot_triggered();
            void on_actionCopy_snapshot_triggered();

            // algorithm menu
            void on_actionRefine_loop_triggered();
            void on_actionCatmull_Clark_triggered();
            void on_actionIsotropic_Remesh_triggered();

            void on_actionMesh_Simplification_triggered();

            //Show Mesh Stats
            void on_actionMesh_Statistics_triggered();


            // view menu
            void on_actionView_polyhedron_triggered();

private:
        Scene* m_pScene;
        Viewer* m_pViewer;
        Ui::MainWindow* ui;
};

#endif // ifndef MAINWINDOW_H
