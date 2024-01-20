#include "vectorrace.h"

#include <iostream>
#include <QtGui>
using namespace std;

/**
 * \brief Bézier curve through Points.
 * \param ps Spanning Points of the Bézier curve
 * \param t position parameter (0 -> ps[0], 1 -> ps.last)
 * \return the interpolated Point on the curve.
 */
Point bezier(const vector<Point>& ps, double t) {
    if (ps.size()==1)
        return ps[0];
    vector<Point> qs(ps.size()-1);
    for (int i=0; i+1<ps.size(); i++) {
        qs[i] = ps[i].affine(t, ps[i+1]);
    }
    while (qs.size()>1) {
        vector<Point> q1s(qs.size()-1);
        for (int i=0; i+1<qs.size(); i++) {
            q1s[i] = qs[i].affine(t, qs[i+1]);
        }
        qs = q1s;
    }
    return qs[0];
}

Rect computeRange(const vector<Point>& route) {
    double x0 = route[0].x;  double x1 = x0;
    double y0 = route[0].y;  double y1 = y0;
    const Point& p1 = route[route.size()-1];
    if (p1.x<x0)
        x0 = p1.x;
    else if (x1<p1.x)
        x1 = p1.x;
    if (p1.y<y0)
        y0 = p1.y;
    else if (y1<p1.y)
        y1 = p1.y;
    return Rect::of(x0-1, y0-1, x1+1, y1+1);
}

VectorRace::VectorRace(QWidget *parent) :QWidget(parent) {
    setMinimumSize(300, 300);
    setMaximumSize(1800, 1800);
    setWindowTitle(tr("Vector Race!"));
    this->range = computeRange(route);
}

VectorRace::~VectorRace() = default;

Rect computeScale(const Rect& range, int width, int height) {
    const double dx = min(width/range.dx, height/range.dy);
    return { range.x0 -(width/dx-range.dx)/2, range.y1() +(height/dx-range.dy), dx, -dx };
}

void VectorRace::paintEvent(QPaintEvent*) {
    this->scale = computeScale(range, width(), height());
    QPainter p(this);
    p.setPen(Qt::black);
    const Point& p0 = route[0];
    int lastX = scale.px(p0.x);  int lastY = scale.py(p0.y);
    for (double t=0.0; t<=1.0; t+=0.01) {
        const Point p1 = bezier(route, t);
        int px = scale.px(p1.x);  int py = scale.py(p1.y);
        p.drawLine(lastX,lastY, px,py);
        lastX = px;  lastY = py;
    }
    p.end();
}

void VectorRace::evolve() {
    for (uint p=0; p<positions.size(); p++)
        positions[p].translate(velocities[p]);
    turn = 0;
}

void VectorRace::proceed() {
    if (turn>=positions.size())
        return;
    velocities[turn] += {ax, ay};
    turn++;
    if (turn>=positions.size())
        evolve();
    repaint();
}

void VectorRace::accelerateUp() {
    ay += 1.0;
    if (ay>=1.0)
        ay = 1.0;
}

void VectorRace::accelerateDown() {
    ay -= 1.0;
    if (ay<=-1.0)
        ay = -1.0;
}

void VectorRace::accelerateLeft() {
    ax -= 1.0;
    if (ax<=-1.0)
        ax = -1.0;
}

void VectorRace::accelerateRight() {
    ax += 1.0;
    if (ax>=1.0)
        ax = 1.0;
}

void VectorRace::keyPressEvent(QKeyEvent* key_event) {
    switch (key_event->key()) {
        case Qt::Key_Up:
            accelerateUp();
        break;
        case Qt::Key_Down:
            accelerateDown();
        break;
        case Qt::Key_Left:
            accelerateLeft();
        break;
        case Qt::Key_Right:
            accelerateRight();
        break;
        case Qt::Key_Enter:
            proceed();
        break;
        default:
            cout<<"Unknown key: "<<(key_event->key())<<" '"<<(key_event->text().toStdString())<<"'";
    }
    key_event->accept();
}
