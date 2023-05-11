//
// Created by hs on 22. 10. 27.
//

#include <canine_simulation/SimulMain.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    StartSimulation();
    StartCommunication();

    return a.exec();
}


//int main()
//{
//    StartSimulation();
//    sleep(10000000);
//}