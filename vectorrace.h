#ifndef VECTORRACE_H
#define VECTORRACE_H

#include <QWidget>
#include <vector>
#include <random>
#include <set>

typedef unsigned char  byte;

struct Vector;

struct Point {
    double x, y;

    const static Point ORIGIN;

    bool operator <(const Point& p) const {
        return y<p.y || y==p.y&&x<p.x;
    }

    [[nodiscard]]
    Point translate(const Vector& v) const;

    [[nodiscard]]
    Vector operator -(const Point& p) const;

    [[nodiscard]]
    Point interpolate(double t, const Point& p) const;

    Point scale(double f) const;
};

struct Vector {
    double x, y;
    const static Vector ZERO;

    Vector operator+(const Vector& v) const {
        return  { x+v.x, y+v.y };
    }

    Vector operator*(double f) const {
        return { x*f, y*f };
    }

    Vector& operator+=(const Vector& v) {
        x += v.x;  y+= v.y;
        return *this;
    }

    [[nodiscard]]
    double norm2() const {
        return x*x +y*y;
    }

    [[nodiscard]]
    Vector normed() const {
        double f = 1/sqrt(norm2());
        return { x*f, y*f };
    }

    [[nodiscard]]
    Vector perp() const {
        return { -y, x };
    }

    [[nodiscard]]
    Vector operator-() const {
        return { -x, -y };
    }

    [[nodiscard]]
    Vector rotate(double angle) const {
        return rotate(cos(angle), sin(angle));
    }

    [[nodiscard]]
    Vector rotate(double c, double s) const {
        return {c*x-s*y, s*x+c*y };
    }
};

inline Point Point::translate(const Vector& v) const {
    return { x + v.x,  y + v.y };
}

inline Vector Point::operator -(const Point& p) const {
    return { x-p.x, y-p.y };
}

inline Point Point::interpolate(double t, const Point& p) const {
    return Point(*this).translate((p-*this)*t);
}

inline Point Point::scale(double f) const {
    return ORIGIN.translate((*this-ORIGIN)*f);
}

Point bezier(const std::vector<Point>& ps, double t=0.5);

struct Circle {
    Point center;
    double radius; // >0
    [[nodiscard]]
    std::vector<Point> intersect(const Circle& circle) const;
};

struct Rect {
    double x0, y0, dx, dy;
    static Rect of(double x0, double y0, double x1, double y1) {
        return {x0,y0, x1-x0, y1-y0};
    }

    [[nodiscard]]
    double y1() const {
        return y0+dy;
    }

    [[nodiscard]]
    int px(double x) const {
        return int(lround((x-x0)*dx));
    }

    [[nodiscard]]
    int py(double y) const {
        return int(lround((y-y0)*dy));
    }
};

class VectorRace : public QWidget {
    Q_OBJECT

public:
    explicit VectorRace(QWidget *parent = nullptr);
    ~VectorRace() override;

    void paintEvent(QPaintEvent*) override;
    void keyPressEvent(QKeyEvent*) override;
    const QColor bgColor = QColor(232, 232, 232);

    [[nodiscard]]
    uint numPlayers() const {
        return positions.size();
    }

    const static std::vector<QColor> colors;

protected:
    void placeCar(QPainter& p, Point const& pos, Vector const& v, unsigned n) const;

    void evolve();
    // void drawWinning(QPainter& p);
    void accelerateLeft();
    void accelerateRight();
    void accelerateUp();
    void accelerateDown();
    void proceed();
    void closeSplash();
    void reset();
    void paintRoute(QPainter& p) const;
    void paintPost(QPainter& p, const Point& p0, const Point& p1, const QString &label) const;

    void loadImages();
    QPixmap* car = nullptr;
    QPixmap* carLeft = nullptr;
    QPixmap* carUp = nullptr;
    QPixmap* carDown = nullptr;
    double w = 0.05;
    QPixmap* splashScreen = nullptr;

private:
    const static std::vector<Point> route0;
    std::vector<Point> route;
    std::vector<Point> positions;
    std::vector<Vector>  velocities;
    double ax = 0.0, ay = 0.0;
    uint step = 0;
    uint turn = 0;
    std::string status;
    std::vector<uint> winners;
    Rect range = {};
    Rect scale = {};
    enum State {
        READY, STARTING, GAME_OVER
    };
    State state;
    QTimer* timer;
};
#endif // VECTORRACE_H
