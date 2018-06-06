#ifndef RIGID_BODY_GUI_WIDGET_H
#define RIGID_BODY_GUI_WIDGET_H

#include <QTimer>
#include <QGLWidget>
#include <QApplication>
#include <QMouseEvent>

namespace rigid_body
{
  namespace gui
  {

    class Widget
    : public QGLWidget
    {
      Q_OBJECT

    private:

      //QOpenGLContext m__context;  // QWindow way

      QTimer        m_timer;

    public:

      Widget( QGLFormat const & format, QWidget *parent = 0);
      ~Widget();

      QSize minimumSizeHint() const;
      QSize sizeHint() const;

      public slots:

      void animate();

      //signals:

    protected:

      void initializeGL();
      void paintGL();
      void resizeGL(int width, int height);
      void mousePressEvent(QMouseEvent *event);
      void mouseMoveEvent(QMouseEvent *event);
      void mouseReleaseEvent(QMouseEvent *event);
      void keyPressEvent(QKeyEvent *e);
      
    };
    
  }// namespace gui
}// namespace rigid_body

// RIGID_BODY_GUI_WIDGET_H
#endif
