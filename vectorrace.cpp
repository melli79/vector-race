#include "vectorrace.h"

#include <iostream>
#include <QApplication>
#include <QtGui>

const double epsilon = 1e-16;
double sqr(double x) {
    return x*x;
}

const Point Point::ORIGIN = { 0.0, 0.0 };
const Vector Vector::ZERO = { 0.0, 0.0 };
/**
 * \brief Bézier curve through Points.
 * \param ps Supporting Points of the Bézier curve
 * \param t position parameter (0 -> ps[0], 1 -> ps.last)
 * \return the interpolated Point on the curve.
 */
Point bezier(const std::vector<Point>& ps, double t) {
    if (ps.size()==1)
        return ps[0];
    std::vector<Point> qs(ps.size()-1);
    for (int i=0; i+1<ps.size(); i++) {
        qs[i] = ps[i].interpolate(t, ps[i+1]);
    }
    while (qs.size()>1) {
        std::vector<Point> q1s(qs.size()-1);
        for (int i=0; i+1<qs.size(); i++) {
            q1s[i] = qs[i].interpolate(t, qs[i+1]);
        }
        qs = std::move(q1s);
    }
    return qs[0];
}

Rect computeRange(const std::vector<Point>& route) {
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
    double dx=x1-x0,  dy=y1-y0;
    return { x0-0.1*dx, y0-0.1*dy, 1.2*dx, 1.2*dy };
}

QPixmap* loadCar() {
    QPixmap* car = new QBitmap(24,12);
    auto p = QPainter(car);
    auto br = QBrush(Qt::transparent);
    p.fillRect(0,0, 24,12, br);
    p.setPen(Qt::black);
    p.setFont(QFont("Arial", 16, QFont::Bold));
    p.drawText(0,12, ">>");
    p.end();
    return car;
}

QPixmap* loadLeftCar(QPixmap* car) {
    QPixmap* leftCar = new QBitmap(24,12);
    auto p = QPainter(leftCar);
    auto br = QBrush(Qt::transparent);
    p.fillRect(0,0, 24,12, br);
    p.translate(24, 0);
    p.rotate(180);
    p.setPen(Qt::black);
    p.setFont(QFont("Arial", 16, QFont::Bold));
    p.drawText(0,0, ">>");
    p.end();
    return leftCar;
}

QPixmap* loadUpCar(QPixmap* car) {
    QPixmap* upCar = new QBitmap(12,24);
    auto p = QPainter(upCar);
    auto br = QBrush(Qt::transparent);
    p.fillRect(0,0, 12, 24, br);
    p.translate(12,24);
    p.rotate(-90);
    p.setPen(Qt::black);
    p.setFont(QFont("Arial", 16, QFont::Bold));
    p.drawText(0,0, ">>");
    p.end();
    return upCar;
}

QPixmap* loadDownCar(QPixmap* car) {
    QPixmap* downCar = new QBitmap(12,24);
    auto p = QPainter(downCar);
    auto br = QBrush(Qt::transparent);
    p.fillRect(0,0, 24,12, br);
    p.rotate(90);
    p.setPen(Qt::black);
    p.setFont(QFont("Arial", 16, QFont::Bold));
    p.drawText(0,0, ">>");
    p.end();
    return downCar;
}

void VectorRace::loadImages() {
    car = loadCar();
    carLeft = loadLeftCar(car);
    carUp = loadUpCar(car);
    carDown = loadDownCar(car);
}

const std::vector<Point> VectorRace::route0 = {{0.0, 0.0}, {1.5,0.0}, {1.5,1.0}, {-0.5,0.0},
       {-0.5,1.0}, {1.0,1.0}};

std::vector<Point> scaleRoute(std::vector<Point> const& route0, double w, unsigned numPlayers) {
    std::vector<Point> route;  route.reserve(route0.size());
    double f = double(numPlayers/2+1)/(w);
    for (auto const& p : route0)
        route.emplace_back(p.scale(f));
    return route;
}

VectorRace::VectorRace(QWidget* parent) :QWidget(parent), state(STARTING) {
    setMinimumSize(300, 600);
    setMaximumSize(1800, 2400);
    setWindowTitle("Vector Race!");
    const ushort numPlayers = 3;
    route = scaleRoute(route0, w, numPlayers);
    w = double(numPlayers/2+1);
    positions.resize(numPlayers);  velocities.resize(numPlayers);
    this->range = computeRange(route);
    this->timer = new QTimer(this);
    timer->callOnTimeout(this, QOverload<>::of(&VectorRace::closeSplash));
    loadImages();
    reset();
}

void VectorRace::reset() {
    auto const& start = route[0];
    auto v = (route[1]-start).perp().normed();
    unsigned k=0;
    for (int p=-int(positions.size()/2); p<=int(positions.size()/2); ++p,++k)
        if (p!=0||positions.size()%2==1) {
            positions[k] = start.translate(v*p);
            velocities[k] = Vector::ZERO;
        }
    ax = 0.0;  ay = 0.0;
    state = STARTING;
    timer->start(3000);
    update();
}

void VectorRace::closeSplash() {
    state = READY;
}

VectorRace::~VectorRace() {
    delete car;  delete carLeft;
    delete carUp;  delete carDown;
}

Rect computeScale(const Rect& range, int width, int height) {
    const double dx = std::min(width/range.dx, height/range.dy);
    return { range.x0 -(width/dx-range.dx)/2, range.y1() +(height/dx-range.dy), dx, -dx };
}

std::vector<Point> Circle::intersect(const Circle& c2) const {
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

const std::vector<QColor> VectorRace::colors = { Qt::green, Qt::cyan, QColor(255,0,255), QColor(192,128,0), Qt::blue, Qt::red };

void VectorRace::placeCar(QPainter& p, Point const& pos, Vector const& v, unsigned n) const {
    QPixmap const* car;
    int offsetX=0,  offsetY=0;
    if (abs(v.x)>=abs(v.y)) { // car horizontal
        if (v.x>=0) {
            car = this->car;
            offsetX = -car->width();
            offsetY = -car->height()/2;
        } else {
            car = this->carLeft;
            offsetY = -car->height()/2;
        }
    } else // car vertical
        if (v.y>0) {
            car = this->carUp;
            offsetX = -car->width()/2;
        } else {
            car = this->carDown;
            offsetX = -car->width()/2;
            offsetY = -car->height();
        }
    p.setBackground(Qt::transparent);
    p.setPen(colors[n%colors.size()]);
    int px = scale.px(pos.x),  py=scale.py(pos.y);
    p.drawPixmap(px+offsetX, py+offsetY, *car);
    int vx=int(lround(scale.dx*v.x)),  vy=int(lround(scale.dy*v.y));
    p.drawLine(px, py, px+vx, py+vy);
    p.drawLine(px+int(0.9*vx-0.1*vy), py+int(0.1*vx+vy), px+vx,py+vy);
    p.drawLine(px+int(0.9*vx-0.1*vy), py+int(-0.1*vx+vy), px+vx,py+vy);
    p.drawLine(px+vx,py+vy, px+2*vx,py+2*vy);
    p.drawLine(px+int(1.9*vx-0.1*vy), py+int(0.1*vx+2*vy), px+2*vx,py+vy);
    p.drawLine(px+int(1.9*vx-0.1*vy), py+int(-0.1*vx+2*vy), px+2*vx,py+2*vy);
}

void VectorRace::paintRoute(QPainter& p) const {
    p.setPen(Qt::black);
    const double dt = 0.002;
    Point p0 = route[0];
    const Point& p01 = bezier(route, -dt);
    const auto ps0 = Circle { p0, w }.intersect(Circle { p01, w });
    assert(ps0.size()==2);
    Point lastL = ps0[0];  Point lastR = ps0[1];
    for (double t=dt; t<=1.0+dt+epsilon; t+=dt) {
        const Point p1 = bezier(route, t);
        const auto ps = Circle { p0, w }.intersect(Circle { p1, w });
        assert(ps.size()==2);
        Point pL = ps[0], pR = ps[1];
        if ((pL-lastL).norm2()>sqr(w)) {
            // Points need to be swapped
            std::swap(pL, pR);
        }
        p.drawLine(scale.px(lastL.x),scale.py(lastL.y), scale.px(pL.x),scale.py(pL.y));
        p.drawLine(scale.px(lastR.x),scale.py(lastR.y), scale.px(pR.x),scale.py(pR.y));
        p0 = p1;
        lastL = pL;  lastR = pR;
    }
    paintPost(p, p01, route[0], "Start");
    paintPost(p, route[route.size()-1], bezier(route, 1+dt), "Finish");
}

void VectorRace::paintEvent(QPaintEvent*) {
    this->scale = computeScale(range, width(), height());
    QPainter p(this);
    paintRoute(p);
    unsigned n=0;
    for (auto const& pos : positions) {
        auto v = velocities[n];
        if (n==turn)
            v += {ax, ay};
        placeCar(p, pos, v, n++);
    }
    auto p0 = positions[turn].translate(velocities[turn]);
    p.setPen(colors[turn%colors.size()]);
    for (short y=-1; y<=1; ++y) {
        p.drawLine(scale.px(p0.x-1), scale.py(p0.y+y), scale.px(p0.x), scale.py(p0.y+y));
        p.drawLine(scale.px(p0.x), scale.py(p0.y+y), scale.px(p0.x+1), scale.py(p0.y+y));
    }
    for (short x=-1; x<=1; ++x) {
        p.drawLine(scale.px(p0.x+x), scale.py(p0.y-1), scale.px(p0.x+x), scale.py(p0.y));
        p.drawLine(scale.px(p0.x+x), scale.py(p0.y), scale.px(p0.x+x), scale.py(p0.y+1));
    }
    p.setPen(Qt::black);
    p.setFont(QFont("Arial", 12));
    p.drawText(6,12, QString("%1's turn").arg(turn+1));
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
    for (unsigned p=0; p<positions.size(); ++p)
        positions[p] = positions[p].translate(velocities[p]);
    turn = 0;
}

void VectorRace::proceed() {
    if (turn>=positions.size())
        return;
    velocities[turn] += {ax, ay};
    ax = 0.0;  ay = 0.0;
    ++turn;
    if (turn>=positions.size())
        evolve();
    update();
}

void VectorRace::accelerateUp() {
    ay += 1.0;
    if (ay>=1.0)
        ay = 1.0;
    update();
}

void VectorRace::accelerateDown() {
    ay -= 1.0;
    if (ay<=-1.0)
        ay = -1.0;
    update();
}

void VectorRace::accelerateLeft() {
    ax -= 1.0;
    if (ax<=-1.0)
        ax = -1.0;
    update();
}

void VectorRace::accelerateRight() {
    ax += 1.0;
    if (ax>=1.0)
        ax = 1.0;
    update();
}

void VectorRace::keyPressEvent(QKeyEvent* key_event) {
    switch (key_event->key()) {
        case Qt::Key_Escape:
            QApplication::exit(0);
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
        case Qt::Key_Return:
            proceed();
        break;
        default:
            std::cout<<"Unknown key: "<<(key_event->key())<<" '"<<(key_event->text().toStdString())<<"'";
    }
    key_event->accept();
}
