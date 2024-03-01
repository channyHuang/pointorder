#include <QCoreApplication>

#include "parseBag.h"
#include <iostream>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    std::cout << "start parse bag" << std::endl;
    ParseBag* parser = ParseBag::getInstance();
    parser->parseBag("D:/dataset/lab/c2_lvi/20230630-obs-lvi.bag");

    std::cout << "end" << std::endl;
    return a.exec();
}
