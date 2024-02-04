#include "vectorrace.h"

#include <iostream>
#include <QtGui>
using namespace std;

const double epsilon = 1e-16;
double sqr(double x) {
    return x*x;
}

/**
 * \brief Bézier curve through Points.
 * \param ps Supporting Points of the Bézier curve
 * \param t position parameter (0 -> ps[0], 1 -> ps.last)
 * \return the interpolated Point on the curve.
 */
Point bezier(const vector<Point>& ps, double t) {
    if (ps.size()==1)
        return ps[0];
    vector<Point> qs(ps.size()-1);
    for (int i=0; i+1<ps.size(); i++) {
        qs[i] = ps[i].interpolate(t, ps[i+1]);
    }
    while (qs.size()>1) {
        vector<Point> q1s(qs.size()-1);
        for (int i=0; i+1<qs.size(); i++) {
            q1s[i] = qs[i].interpolate(t, qs[i+1]);
        }
        qs = move(q1s);
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
    return Rect::of(x0-0.2, y0-0.2, x1+0.2, y1+0.2);
}

void VectorRace::loadImages() {
}

VectorRace::VectorRace(QWidget* parent) :QWidget(parent), state(STARTING) {
    setMinimumSize(300, 600);
    setMaximumSize(1800, 2400);
    setWindowTitle("Vector Race!");
    this->range = computeRange(route);
    this->timer = new QTimer(this);
    timer->callOnTimeout(this, QOverload<>::of(&VectorRace::closeSplash));
    loadImages();
    reset();
}

void VectorRace::reset() {
    state = STARTING;
    timer->start(3000);
}

void VectorRace::closeSplash() {
    state = READY;
}

VectorRace::~VectorRace() = default;

Rect computeScale(const Rect& range, int width, int height) {
    const double dx = min(width/range.dx, height/range.dy);
    return { range.x0 -(width/dx-range.dx)/2, range.y1() +(height/dx-range.dy), dx, -dx };
}

vector<Point> Circle::intersect(const Circle& c2) const {
    Vector v = c2.center - center;
    double d = sqrt(v.norm2());
    if (d < epsilon || d < abs(radius - c2.radius) || radius+c2.radius < d)
        return {};
    double l = (sqr(d) + sqr(radius) -sqr(c2.radius)) / d;
    double h2 = sqr(radius) -sqr(l);
    if (h2 < 3*epsilon*(radius*abs(l)+1.0)) {
        auto p = Point(center);
        return { p.translate(v.normed()*l) };
    }
    const auto e = v*(1/d);
    const auto b = center.translate(e*l);
    const auto p = e.perp() * sqrt(h2);
    return { b.translate(p), b.translate(-p) };
}


void VectorRace::paintEvent(QPaintEvent*) {
    this->scale = computeScale(range, width(), height());
    QPainter p(this);
    p.setPen(Qt::black);
    const double dt = 0.002;
    Point p0 = route[0];
    const Point& p01 = bezier(route, dt);
    const auto ps0 = Circle { p0, w }.intersect(Circle { p01, w });
    Point lastL = ps0[0];  Point lastR = ps0[1];
    for (double t=dt; t<=1.0+dt+epsilon; t+=dt) {
        const Point p1 = bezier(route, t);
        const auto ps = Circle { p0, w }.intersect(Circle { p1, w });
        Point pL = ps[0], pR = ps[1];
        if ((pL-lastL).norm2()>sqr(w)) {
            // Points need to be swapped
            swap(pL, pR);
        }
        p.drawLine(scale.px(lastL.x),scale.py(lastL.y), scale.px(pL.x),scale.py(pL.y));
        p.drawLine(scale.px(lastR.x),scale.py(lastR.y), scale.px(pR.x),scale.py(pR.y));
        p0 = p1;
        lastL = pL;  lastR = pR;
    }
    paintPost(p, route[0], p01, "Start");
    paintPost(p, bezier(route, 1-dt), route[route.size()-1], "Finish");
    p.end();
}

void VectorRace::paintPost(QPainter& p, const Point& p0, const Point& p1, const QString& label) const {
    const auto n = (p1-p0).normed().perp()*w;
    const Point b = p0.translate(-n);
    const Point t = p0.translate(n);
    p.setPen(Qt::red);
    p.setFont(QFont("arial", 16, QFont::Bold));
    p.drawLine(scale.px(b.x), scale.py(b.y), scale.px(t.x), scale.py(t.y));
    p.drawText(scale.px(t.x), scale.py(t.y)-2, label);
}

void VectorRace::evolve() {
    for (uint p=0; p<positions.size(); p++)
        positions[p] = positions[p].translate(velocities[p]);
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
