#include "vectorrace.h"

#include <QApplication>

int main(int nArgs, char* args[])
{
    QApplication app(nArgs, args);

    VectorRace w;
    w.show();
    return app.exec();
}
