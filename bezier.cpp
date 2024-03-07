//
// Created by Melchior Grützmann on 2024-03-04.
//

#include "bezier.h"

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

Position findPosition(std::vector<Point> const& route, Point const& p) {
    double dt0 = sqrt(epsilon);
    double tOpt = 0.0,  d2min = (p-bezier(route, tOpt)).norm2();
    for (double t=0.0; t<=1.0+epsilon; t+=0.02) {
        double d2 = (p-bezier(route, t)).norm2();
        if (d2<d2min) {
            d2min = d2;  tOpt = t;
        }
    }
    double lastT = tOpt>=0.01 ? tOpt-0.01 : 0.01;
    double lastOff2 = (p-bezier(route, lastT)).norm2();
    while (abs(tOpt-lastT) > dt0) {
        double dt = tOpt - lastT;
        double dy = d2min - lastOff2;
        if (abs(dy) < 5*dt0)
            break;
        double t1 = tOpt -(dt/2)*d2min/dy;
        lastT = tOpt;  lastOff2 = d2min;
        tOpt = t1;  d2min = (p-bezier(route, tOpt)).norm2();
        if (lastOff2<d2min) {
            tOpt = lastT;  d2min = lastOff2;
            break;
        }
    }
    auto p0 = bezier(route, tOpt);
    int s = sgn((p-p0).dot((bezier(route, tOpt+dt0) -p0).perp()));
    return { tOpt, s*sqrt(d2min) };
}

double computeLength(std::vector<Point> const& route, double end) {
    double len = 0.0;
    Point lastPos = route[0];
    double dt = 0.001;
    for (double t=0; t<=end+epsilon; t+=dt) {
        Point pos = bezier(route, t);
        len += sqrt((pos-lastPos).norm2());
        lastPos = pos;
    }
    return len;
}
