#include <QCoreApplication>

#include "device.h"
//#include "KeyValueParser.h"
using namespace std;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    //Declare instance of zaberLightPositoiner device class
    zaberLightPositioner zlp;

    //Attempt to detect that devices are connected
    zlp.Detect_Device();

    //Issue commands to the device
    // For example, set the rotary device to 50.0 degrees azimuth
    // wait until the device is not busy and move back to 0.0 degrees
    while(zlp.Is_Device_Busy());
    cout << "Moving rotation stage to 10.00 degrees azimuth." << endl;
    zlp.Set_Rotation_Stage_Angular_Position(90.0);

    while(zlp.Is_Device_Busy()) cout << "Waiting for device to move." << endl;

    cout << "Moving rotation stage back to 0.00 degrees azimuth." << endl;
    zlp.Set_Rotation_Stage_Angular_Position(0.0);


    while(zlp.Is_Device_Busy()) cout << "Waiting for device to move." << endl;

    //elelvation testing
    cout << "testing elevation movement" << endl;
    zlp.Set_Stepper_Motor_Angular_Position(0.0);
    while(zlp.Is_Device_Busy()) cout << "Waiting for device to move." << endl;
    cout << "return elevation motor" << endl;
    zlp.Set_Stepper_Motor_Angular_Position(80.0);
    while(zlp.Is_Device_Busy()) cout << "Waiting for device to move." << endl;


    cout << "Ending program." << endl;

    return a.exec();
}


// Commandline interface for manual commands
void cmd_loop()
{
    string command;

    cout << "Command syntax: " << endl;
    cout << "For azimuth control: 'setaz [angle in degrees]'" << endl;
    cout << "For elevation control: 'setel [angle in degrees]'" << endl;

    while (command != "q" && command != "quit" && command != "exit") {
        // gather user input for commands each loop
        cin >> command;
        cout << "Current positions: " << endl;
        cout << "" << endl;

        // TODO: Display positions of both motors before each command

        if (command.rfind("setaz", 0) == 0) {
            // set azimuth angle command
        }
        if (command.rfind("setel", 0) == 0) {
            // set elevation angle command
        }
        if (command.rfind("setaz_accel", 0) == 0) {
            // set azimuth acceleration parameter
        }
        if (command.rfind("setel_accel", 0) == 0) {
            // set elevation acceleration parameter
        }

        // Additional command options below...

    }





}
