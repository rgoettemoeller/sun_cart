#include "device.h"
#include <iostream>
#include <sstream>
#include <QDateTime>
#include <chrono>
#include <ctime>

#define UNIX 1 // comment this out if on windows

#ifdef UNIX
#define SERIAL_NUMBER "A10JK57L"
#else
#define SERIAL_NUMBER "A10JK57LA"
#endif

#define STEPPER_ACCELERATION 1000
#define STEPPER_MOTOR_ACCELERATION 10.0
#define STEPPER_MOTOR_MULTIPLIER 10.0
#define STEPPER_MOTOR_VELOCITY 10.0
#define ROTATION_STAGE_VELOCITY 10.0
#define ROTATION_STAGE_ACCELERATION 10.0

/***********************************************************************************
 * device Class Member Functions
 */

//Default constructor for empty device
device::device(){
    device_name = "";
    device_manufacturer = "";
    device_model = "";
    device_type = no_device_type;
    device_subtype = no_device_subtype;
    device_initialized = false;
    device_detected = false;
    device_busy = false;
}



void device::Detect_Device(){
    if(console_output) cout << "Base device class cannot detect device." << endl;
    device_detected = false;
}

void device::Initialize_Device(){
    if(console_output) cout << "Base device class cannot be initialized." << endl;
    device_initialized = false;
}

string device::Get_Timestamp(bool filename_safe){

    QDateTime ts = QDateTime::currentDateTime();
    int year = ts.date().year();
    int month = ts.date().month();
    int day = ts.date().day();
    int hour = ts.time().hour();
    int min = ts.time().minute();
    int sec = ts.time().second();

    stringstream ss;
    if(filename_safe){
        ss << year << "-";
        if(month<10) ss << "0" << month << "-";
        else ss << month << "-";
        if(day<10) ss << "0" << day << "-";
        else ss << day << "_";
        if(hour<10) ss << "0" << hour << "-";
        else ss << hour << "-";
        if(min<10) ss << "0" << min << "-";
        else ss << min << "-";
        if(sec<10) ss << "0" << sec;
        else ss << sec;
    }
    else{
        ss << year << "-";
        if(month<10) ss << "0" << month << "-";
        else ss << month << "-";
        if(day<10) ss << "0" << day << " ";
        else ss << day << " ";
        if(hour<10) ss << "0" << hour << ":";
        else ss << hour << ":";
        if(min<10) ss << "0" << min << ":";
        else ss << min << ":";
        if(sec<10) ss << "0" << sec;
        else ss << sec;
    }
    return ss.str();
}

//Returns the Device Type as its String Enumeration Name
string device::Get_Device_Type_As_String(){
    if(device_type == camera)
        return "camera";
    else if(device_type == motion_control_stage)
        return "motion_control_stage";
    else if(device_type == calibration_source)
        return "calibration_source";
    else return "no_device_type";
}

//Returns the Device Type as its String Enumeration Name
string device::Get_Device_Subtype_As_String(){
    if(device_subtype == visible_mono)
        return "visible_mono";
    else if(device_subtype == visible_color)
        return "visible_color";
    else if(device_subtype == visible_mono_dot_polarimeter)
        return "visible_mono_dot_polarimeter";
    else if(device_subtype == visible_mono_microgrid_polarimeter)
        return "visible_mono_microgrid_polarimeter";
    else if(device_subtype == visible_color_microgrid_polarimeter)
        return "visible_color_microgrid_polarimeter";
    else if(device_subtype == rotary_stage)
        return "rotary_stage";
    else if(device_subtype == rotary_stepper_stages)
        return "rotary_stepper_stages";
    else if(device_subtype == translation_stage)
        return "translation_stage";
    else if(device_subtype == robotic_arm)
        return "robotic_arm";
    else if(device_subtype == integrating_sphere)
        return "integrating_sphere";
    else if(device_subtype == blackbody)
        return "blackbody";
    else return "no_device_subtype";
}

















/***********************************************************************************
 * zaberLightPositioner Class Member Functions
 */

//Constructor
zaberLightPositioner::zaberLightPositioner(){

    //Set default device parameters
    device_name = "Light Positioner";
    device_manufacturer = "Zaber";
    device_model = "X-RST120AK-E03 + X-NMS17-E01";
    device_type = motion_control_stage;
    device_subtype = rotary_stepper_stages;
    zaber_serial_port_name = "";
    irblaster_serial_port_name = "";
}

//Thread that will make multiple attempt to connect to a device
void zaberLightPositioner::Detect_Device(){

    cout << "================================================" << endl;
    cout << "| Attempting to Detect Zaber Light Positioner: |" << endl;
    cout << "================================================" << endl;

    //Attempt to update zaber device database (connect to internet)
    cout << " ->Attempting to update Zaber device database..." << std::endl;
    Library::enableDeviceDbStore();

    //Detection flags
    bool zaberConnected = false;
    bool irblasterConnected = true; //set to true to bypass detection

    //Attempt to detect and initialize device. Will make multiple attempts
    for(unsigned int k=0; k<number_of_detection_tries; k++){
        if((!Is_Device_Detected()) && (!Is_Device_Initialized())){

            //Find Zaber serial connection
            bool found = Detect_Serial_Ports();

            if(found){

                //Attempt to connect to the zaber devices through the detected serial port
                cout << " ->Attempting connection to Zaber device(s) on serial port " << zaber_serial_port_name << ". Attempt " << k+1 << "of " << number_of_detection_tries << "." << endl;
                connection = Connection::openSerialPort(zaber_serial_port_name);

                //Look for devices connected to the specified COM port
                deviceList = connection.detectDevices();
                if(deviceList.size()<1){
                    cout << " ->No Zaber devices found. Trying again." << endl;
//                    sleep_until(system_clock::now() + 1s); //Timeout for 1 second
                    continue;
                }
                else if(deviceList.size()==1){
                    cout << " ->Only 1 Zaber device found. Trying again." << endl;
                    continue;
                }
                else if(deviceList.size()>2){
                    cout << " ->Too many Zaber devices found. Trying again." << endl;
                    continue;
                }
                else{ // Ensure that 2 Zaber are found
                    cout << "   ->Found " << deviceList.size() << " Zaber device(s)." << endl << endl;

                    //Set device and device axes to point to local class variables
                    rotation_stage_device = new Device(deviceList[0]);
                    rotation_stage_axis = new Axis(rotation_stage_device->getAxis(1));
                    stepper_motor_device = new Device(deviceList[1]);
                    stepper_motor_axis = new Axis(stepper_motor_device->getAxis(1));

                    //Set up base device parameters
                    cout << "Rotation Stage Information:" << endl;
                    cout << "------------------" << endl;
                    cout << " ->Device Name: " << rotation_stage_device->getName() << endl;
                    cout << " ->Device Address: " << rotation_stage_device->getDeviceAddress() << endl;
                    cout << " ->Device ID: " << rotation_stage_device->getDeviceId() << endl;
                    cout << " ->Device Serial Number: " << rotation_stage_device->getSerialNumber() << endl;
                    cout << " ->" << rotation_stage_device->getFirmwareVersion().toString() << endl;
                    cout << " ->Number of Axes: " << rotation_stage_device->getAxisCount() << endl;
                    cout << "Rotation stage detection successful." << endl << endl;

                    cout << "Stepper Motor Information:" << endl;
                    cout << "------------------" << endl;
                    cout << " ->Device Name: " << stepper_motor_device->getName() << endl;
                    cout << " ->Device Address: " << stepper_motor_device->getDeviceAddress() << endl;
                    cout << " ->Device ID: " << stepper_motor_device->getDeviceId() << endl;
                    cout << " ->Device Serial Number: " << stepper_motor_device->getSerialNumber() << endl;
                    cout << " ->" << stepper_motor_device->getFirmwareVersion().toString() << endl;
                    cout << " ->Number of Axes: " << stepper_motor_device->getAxisCount() << endl;
                    cout << "Stepper motor detection successful." << endl << endl;

                    zaberConnected = true;
                }
/*
                //Attempt to open serial connection to IRBlaster device
                //If successful, configure port with IRBlaster required settings
                cout << endl << " IRBlaster Serial Port Configuration:" << endl;
                cout << " -------------------------" << endl;
                irblaster_serial_port.setPortName(QString::fromStdString(Get_IRBlaster_Serial_Port_Name()));
                irblaster_serial_port.setBaudRate(QSerialPort::Baud115200);
                cout << " ->Baud Rate: 115200" << endl;
                irblaster_serial_port.setDataBits(QSerialPort::Data8);
                cout << " ->Data Bits: 8" << endl;
                irblaster_serial_port.setStopBits(QSerialPort::OneStop);
                cout << " ->Stop Bits: 1" << endl;
                irblaster_serial_port.setParity(QSerialPort::NoParity);
                cout << " ->Parity: None" << endl;
                irblaster_serial_port.setFlowControl(QSerialPort::NoFlowControl);
                cout << " ->Flow Control: None" << endl;
                irblaster_serial_port.open(QIODevice::ReadWrite);
                cout << " ->Mode: Read/Write" << endl << endl;

                if(irblaster_serial_port.isOpen()){
                    irblaster_serial_port_open = true;
                    irblasterConnected = true;
                }
*/

                //If all devices are connected attempt to initialize them
                if(zaberConnected && irblasterConnected){

                    device_detected = true;

                    cout << "Attempting to initialize devices:" << endl;
                    cout << "--------------------------------" << endl;
                    Initialize_Device();
                    if(Is_Device_Initialized()){
                        if(console_output) cout << "Device initialization successful." << endl << endl;
                        break;
                    }
                    else if(console_output) cout << "Device initialization failed." << endl << endl;

                }

            }
        }
        // If device is detected and not initialized, keep trying to initialize the
        // device until successful.
        else if((Is_Device_Detected()) && (!Is_Device_Initialized())){
            // Re-attempt to initialize device
            cout << "Re-attempting to initialize device..." << endl << endl;
            Initialize_Device();
            if(Is_Device_Initialized()){
                cout << "Device initialization successful." << endl << endl;
                break;
            }
            else cout << "Device initialization failed." << endl;
        }
        // If somehow the device is not detected but somehow is initialized set
        // the initialization flag to false.
        else if((!Is_Device_Detected()) && (Is_Device_Initialized())){
            device_initialized = false;
        }
        // Otherwise, all is well when the device is both detected and initialized.
        else{
            break;
        }
    }

    //If number of attempts exhausted and device not detected...
    if(!Is_Device_Detected() & !Is_Device_Initialized()){
        cout << "Device detection timed out. Device not detected or initialized." << endl << endl;
    }

}

//Will search any connected serial port connections and identify my specific zaber rotary device
bool zaberLightPositioner::Detect_Serial_Ports(){

// Zaber Serial Port Connection Information:
// Description: USB Serial Port
// Manufacturer: FTDI
// On Ubuntu the Serial number is: A10JK57L
// On Windows the Serial number is: A10JK57LA
// Vendor Identifier: 403
// Product Identifier: 6001

// IRBlaster USB-Controlled Power Switch for Light Source Information:
// Description: USB Serial Port
// Manufacturer:
// On Ubuntu the Serial number is: none
// Vendor Identifier: 1a86
// Product Identifier: 7523

    bool foundZaberDevice = false;
    bool foundIRBlasterDevice = true; //set to true to bypass detection loop

    //First, we check for all serial port devices that are available.
    //Then, we determine if both the zaber device and power switch are present

    cout << " ->Scanning all serial ports for devices..." << endl << endl;

    const auto serialPortInfos = QSerialPortInfo::availablePorts();

    cout << "Total number of ports available: " << serialPortInfos.count() << endl;

    const QString blankString = "N/A";
    QString portname;
    QString portlocation;
    QString description;
    QString manufacturer;
    QString serialnumber;
    QString vendor;
    QString identifier;

    for (const QSerialPortInfo &serialPortInfo : serialPortInfos) {
        portname = serialPortInfo.portName();
        portlocation = serialPortInfo.systemLocation();
        description = serialPortInfo.description();
        manufacturer = serialPortInfo.manufacturer();
        serialnumber = serialPortInfo.serialNumber();
        vendor = (serialPortInfo.hasVendorIdentifier() ? QByteArray::number(serialPortInfo.vendorIdentifier(), 16) : blankString);
        identifier = (serialPortInfo.hasProductIdentifier() ? QByteArray::number(serialPortInfo.productIdentifier(), 16) : blankString);

        //Output Information to console
        cout << "Detected Serial Port Info:" << endl;
        QTextStream out(stdout);
        out << endl
            << "Port: " << portname << endl
            << "Location: " << portlocation << endl
            << "Description: " << description << endl
            << "Manufacturer: " << manufacturer << endl
            << "Serial number: " << serialnumber << endl
            << "Vendor Identifier: " << vendor << endl
            << "Product Identifier: " << identifier << endl
            << "Busy: " << (serialPortInfo.isBusy() ? "Yes" : "No") << endl << endl;

        //Determine if device is one of the ones we are looking for based upon serial number and manufacturer identifier.
        //If found, set the serial port name and return success.
        //The zaber library will then establish the connection to the device.
        //QSerialPort serialport;
        if((serialnumber == SERIAL_NUMBER) && (identifier=="6001")){
            out << "   ->Zaber rotarty stage and stepper motor found on port " << portlocation << "." << endl;
            Set_Zaber_Serial_Port(portlocation.toStdString());
            foundZaberDevice = true;
        }       
        else if((vendor == "1a86") && (identifier=="7523")){
            out << "   ->IRBlaster USB power switch found on port " << portlocation << "." << endl;
            Set_IRBlaster_Serial_Port(portlocation.toStdString());
            foundIRBlasterDevice = true;
        }

        if(foundZaberDevice && foundIRBlasterDevice) return true;
    }
    if(foundZaberDevice && !foundIRBlasterDevice) cout << " ->IRBlaster device not detected." << endl;
    else if(!foundZaberDevice && foundIRBlasterDevice) cout << " ->Zaber device not detected." << endl;
    else cout << " ->Zaber and IRBlaster devices not detected." << endl;
//    sleep_until(system_clock::now() + 1s); //Timeout for 1 second
    return false;
}


//Performs device initialization. Currently just moves the rotary device to home position.
//Moves the stepper motor to 0 position.
void zaberLightPositioner::Initialize_Device(){

    if(Is_Device_Detected()){
        if(console_output) cout << " ->Initializing devices." << endl;
        try {

            //Home zaber devices
            rotation_stage_axis->home(false);
            stepper_motor_axis->moveAbsolute(0.0,Units::ANGLE_DEGREES);            

            //Turn off IRBlaster power outlet
//            bool state = Set_IRBlaster_State(false);
//            if(state == false) device_initialized = true;

            device_initialized = true; //set to true for now since irBlaster is disabled.

        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
        }
    }
    else{
        if(console_output) cout << " ->Initialization failed. Device(s) not detected." << endl;
    }
}

//Checks whether device is idle and ready for next command. This checks both the rotation stage and stepper motor states.
bool zaberLightPositioner::Is_Device_Busy(){

    if(Is_Device_Detected() & Is_Device_Initialized()){
        return (rotation_stage_axis->isBusy() || stepper_motor_axis->isBusy());
    }
    else{
        if(console_output) cout << "Device busy query failed. Device not detected and initialized." << endl;
        return true;
    }
}


//Will move rotation stage back to home position
//Device must be detected, initialized, and not busy
bool zaberLightPositioner::Home_Rotation_Stage(){

    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){
        if(console_output) cout << " ->Homing rotation stage device." << endl;
        try {
            rotation_stage_axis->home();
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << " ->Homing of rotation stage failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Issues a command to stop any movement of rotation stage
//Device must be detected and initialized
bool zaberLightPositioner::Stop_Rotation_Stage(){

    if(Is_Device_Detected() & Is_Device_Initialized()){
        if(console_output) cout << "-->Stopping rotation stage device... " << endl;
        try {
            rotation_stage_axis->stop();
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Stop of rotation stage failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Will start rotatation of the stage to a specified absolute angle
//Device must be detected, initialized, and idle
//Ensures that the specified azimuth angle is within the constrained angular azimuthal range
bool zaberLightPositioner::Set_Rotation_Stage_Angular_Position(double degrees){

    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){

        //Ensure angle is within the correct range.
        //Also ensure that the direction of rotation does not violate the constrainted region
        if(degrees <= max_abs_azimuth && degrees <= max_con_azimuth && degrees >= min_abs_azimuth && degrees >= min_con_azimuth){
            try {
                if(console_output) cout << "-->Moving rotation stage to " << degrees << " degrees (Absolute)." << endl;
                rotation_stage_axis->moveAbsolute(degrees, Units::ANGLE_DEGREES,false, ROTATION_STAGE_VELOCITY, Units::ANGULAR_VELOCITY_DEGREES_PER_SECOND, ROTATION_STAGE_ACCELERATION, Units::ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARED);
                return true;
            } catch (const MotionLibException& e) {
                std::cerr << e.getMessage() << std::endl;
                return false;
            }
        }
        else{
            if(console_output) cout << "-->Specified azimuth angle " << degrees << " degrees is out of range (" << min_con_azimuth << ", " << max_con_azimuth << ") degrees" << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Rotation stage device rotation (absolute) failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Will start rotation of the stage by a specified relative angular amount
//Device must be detected, initialized, and idle
//Ensures that the specified azimuth angle is within the constrained angular azimuthal range
bool zaberLightPositioner::Set_Rotation_Stage_Relative_Angular_Position(double degrees){

    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){

        //Ensure angle is within the correct range.
        //Also ensure that the direction of rotation does not violate the constrainted region
        float total_degrees = Get_Rotation_Stage_Angular_Position() + degrees;
        return Set_Rotation_Stage_Angular_Position(total_degrees);
    }
    else{
        if(console_output) cout << "Rotation stage device rotation (relative) failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Sets the constrained max/min azimuthal range. Ensures it is within the max/min absolute range
bool zaberLightPositioner::Set_Rotation_Stage_Constrained_MaxMin_Azimuth(double max, double min){

    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){
        if(min >= min_abs_azimuth && max <= max_abs_azimuth){
            if(console_output) cout << "-->Set constrained max/min azimuth to " << max << "/" << min << " degrees." << endl;
            max_con_azimuth = max;
            min_con_azimuth = min;
            return true;
        }
        else{
            if(console_output) cout << "-->Failed to set constrained max/min azimuth to " << max << "/" << min << " degrees." << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Rotation stage device constrained max/min azimuth set failed. Device not detected and initialized." << endl;
        return false;
    }
}


//Returns the current position of rotation stage in degrees
double zaberLightPositioner::Get_Rotation_Stage_Angular_Position(){

    if(Is_Device_Detected() & Is_Device_Initialized()){
        double pos = rotation_stage_axis->getPosition(Units::ANGLE_DEGREES);
        if(console_output) cout << "-->Current azimuth positition is " << pos << " degrees." << endl;
        return pos;
    }
    else{
        if(console_output) cout << "Rotation stage device position query failed. Device not detected and initialized." << endl;
        return -1;
    }
}


//Returns the maximum angular velocity
double zaberLightPositioner::Get_Rotation_Stage_Maximum_Angular_Velocity(){

    if(Is_Device_Detected() & Is_Device_Initialized()){
        try {

            double temperature = rotation_stage_axis->getSettings().get("driver.temperature");
            if(console_output) cout << "Rotation stage driver temperature [°C]: " << temperature << std::endl;

            double speed = rotation_stage_axis->getSettings().get("maxspeed", Units::ANGULAR_VELOCITY_DEGREES_PER_SECOND);
            if(console_output) cout << "-->Rotation stage maximum rotation speed [deg/s]: " << speed << std::endl;

            return speed;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return 0;
        } catch (...) {
            if(console_output) cout << "Exception Unknown" << endl;
            return -1;
        }
    }
    else{
        if(console_output) cout << "Can't get rotation stage maximum velocity. Device not detected and initialized." << endl;
        return 0;
    }
}




//Calls the home command on the stepper motor device. Currently this may make the device spin forever
//unless the homing sensor is installed.
bool zaberLightPositioner::Home_Stepper_Motor(){

    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){
        if(console_output) cout << " ->Homing stepper motor device." << endl;
        try {
            stepper_motor_axis->home();
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << " ->Homing of stepper motor failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Issues a stop command to the stepper motor device.
bool zaberLightPositioner::Stop_Stepper_Motor(){

    if(Is_Device_Detected() & Is_Device_Initialized()){
        if(console_output) cout << "-->Stopping stepper motor device... " << endl;
        try {
            stepper_motor_axis->stop();
            return true;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Stepper motor stop failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Issues a move command to the stepper motor in absolute degrees. We will need to apply a conversion factor to convert
//from elevation degrees to device degrees once the system is in place. This ensures that the angle is within the
//constrained angular elevation range.
bool zaberLightPositioner::Set_Stepper_Motor_Angular_Position(double degrees){

    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){

        //Ensure angle is within the correct range.
        //Also ensure that the direction of rotation does not violate the constrainted region
        if(degrees <= max_abs_elevation && degrees <= max_con_elevation && degrees >= min_abs_elevation && degrees >= min_con_elevation){
            try {
                if(console_output) cout << "-->Moving stepper motor to " << degrees << " degrees (Absolute)." << endl;
                stepper_motor_axis->moveAbsolute(degrees * stepper_motor_units_to_elevation_degrees, Units::ANGLE_DEGREES,false, STEPPER_MOTOR_VELOCITY, Units::ANGULAR_VELOCITY_DEGREES_PER_SECOND, STEPPER_MOTOR_ACCELERATION, Units::ANGULAR_ACCELERATION_DEGREES_PER_SECOND_SQUARED);
                return true;
            } catch (const MotionLibException& e) {
                std::cerr << e.getMessage() << std::endl;
                return false;
            }
        }
        else{
            if(console_output) cout << "-->Specified elevation angle " << degrees << " degrees is out of range (" << min_con_elevation << ", " << max_con_elevation << ") degrees" << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Stepper motor device rotation (absolute) failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Issues a move command to the stepper motor in relative degrees. We will need to apply a conversion factor to convert
//from elevation degrees to device degrees once the system is in place. Ensures that the total specified elevation angle
//is within the constained elevation angle range.
bool zaberLightPositioner::Set_Stepper_Motor_Relative_Angular_Position(double degrees){

    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){

        //Ensure angle is within the correct range.
        float total_degrees = Get_Stepper_Motor_Angular_Position() + degrees * stepper_motor_units_to_elevation_degrees;
        return Set_Stepper_Motor_Angular_Position(total_degrees);
    }
    else{
        if(console_output) cout << "Rotation stage device rotation (relative) failed. Device not detected and initialized." << endl;
        return false;
    }
}

bool zaberLightPositioner::Set_Stepper_Motor_Constrained_MaxMin_Elevation(double max, double min){

    if(Is_Device_Detected() & Is_Device_Initialized() & !Is_Device_Busy()){
        if(min >= min_abs_elevation && max <= max_abs_elevation){
            if(console_output) cout << "-->Set constrained max/min elevation to " << max << "/" << min << " degrees." << endl;
            max_con_elevation = max;
            min_con_elevation = min;
            return true;
        }
        else{
            if(console_output) cout << "-->Failed to set constrained max/min elevation to " << max << "/" << min << " degrees." << endl;
            return false;
        }
    }
    else{
        if(console_output) cout << "Stepper motor device constrained max/min elevation set failed. Device not detected and initialized." << endl;
        return false;
    }
}

//Returns the current angluar position of the stepper motor. Note that this will likely need to be converted from
//device degrees to elevation degrees!
double zaberLightPositioner::Get_Stepper_Motor_Angular_Position(){

    if(Is_Device_Detected() & Is_Device_Initialized()){
        double pos = (stepper_motor_axis->getPosition(Units::ANGLE_DEGREES)) / stepper_motor_units_to_elevation_degrees;
        if(console_output) cout << "-->Current elevation positition is " << pos << " degrees." << endl;
        return pos;
    }
    else{
        if(console_output) cout << "Stepper motor device position query failed. Device not detected and initialized." << endl;
        return -1;
    }
}

//Returns stepper motor maximum rotational speed and also outputs driver temperature
double zaberLightPositioner::Get_Stepper_Motor_Maximum_Angular_Velocity(){

    if(Is_Device_Detected() & Is_Device_Initialized()){
        try {

            double temperature = rotation_stage_axis->getSettings().get("driver.temperature");
            if(console_output) cout << "Stepper motor driver temperature [°C]: " << temperature << std::endl;

            double speed = rotation_stage_axis->getSettings().get("maxspeed", Units::ANGULAR_VELOCITY_DEGREES_PER_SECOND);
            if(console_output) cout << "-->Stepper motor maximum rotation speed [deg/s]: " << speed << std::endl;

            return speed;
        } catch (const MotionLibException& e) {
            std::cerr << e.getMessage() << std::endl;
            return 0;
        } catch (...) {
            if(console_output) cout << "Exception Unknown" << endl;
            return -1;
        }
    }
    else{
        if(console_output) cout << "Can't get stepper motor maximum velocity. Device not detected and initialized." << endl;
        return 0;
    }

}




//********************************************************************************
// FUNCTIONS FOR IR BLASTER USB POWER OUTLET THAT CONTROLS COLLIMATED LIGHT SOURCE

//Sends a command ON or OFF to the IRBlaster device to set the on/off state
//Since the device doesn't send a response, the device is queried afterwards
//to ensure state was successfully set and true/false is returned.
bool zaberLightPositioner::Send_ON_OFF_Command(bool on){

    QByteArray tx;
    if(on) tx = "ON\n";
    else tx = "OFF\n";
    if(Is_IRBlaster_Serial_Port_Open()){

        const qint64 bytesWritten = irblaster_serial_port.write(tx);

        if (bytesWritten == -1) {
            //if(console_output)
                cout << "Failed to send command to IRBlaster on port " << Get_IRBlaster_Serial_Port_Name() << "." << endl;
        }
        else if (bytesWritten != tx.size()) {
            //if(console_output)
                cout << "Failed to send complete command to IRBlaster on port " << Get_IRBlaster_Serial_Port_Name() << "." << endl;
        }
        else if (!irblaster_serial_port.waitForBytesWritten(5000)) {
            //if(console_output)
                cout << "Command sent to IRBlaster on port " << Get_IRBlaster_Serial_Port_Name() << " timed out or an error occurred." << endl;
        }

        //IRBlaster does not send a response. Instead check to see if state was set successfully
        bool state = Send_QUERY_Command();
        if(state) return true;
        else return false;
    }
    else{
        cout << "Command not sent: IRBlaster device not connected." << endl;
        return "";
    }

}

//Sends a query commend to get the state of the IRBlaster on/off state. Returns
//true for on and false for off.
bool zaberLightPositioner::Send_QUERY_Command(){

    const QByteArray tx = "?\n";
    if(Is_IRBlaster_Serial_Port_Open()){

        const qint64 bytesWritten = irblaster_serial_port.write(tx);

        if (bytesWritten == -1) {
            if(console_output) cout << "Failed to send command to IRBlaster on port " << Get_IRBlaster_Serial_Port_Name() << "." << endl;
        }
        else if (bytesWritten != tx.size()) {
            if(console_output) cout << "Failed to send complete command to IRBlaster on port " << Get_IRBlaster_Serial_Port_Name() << "." << endl;
        }
        else if (!irblaster_serial_port.waitForBytesWritten(5000)) {
            if(console_output) cout << "Command sent to IRBlaster on port " << Get_IRBlaster_Serial_Port_Name() << " timed out or an error occurred." << endl;
        }

        //Decode response
        bool got_data = false;
        QByteArray readData = irblaster_serial_port.readAll();
        while(!got_data){
            while (irblaster_serial_port.waitForReadyRead(100))
            readData.append(irblaster_serial_port.readAll());
            if(readData.size()>0) got_data=true;
        }

        //Examine response from device
        QByteArray resp = getSubstring(readData,0,1);
        if(resp == "ON") return true;
        else return false;
    }
    else{
        cout << "Command not sent: IRBlaster device not connected." << endl;
        return false;
    }
}



//Extract substrings from a QByteArray.
QByteArray zaberLightPositioner::getSubstring(const QByteArray qstring,size_t start, size_t end){

    if((start<0) || (start>=qstring.size()) || (start>end)) return "";
    else if((end<0) || (end>=qstring.size()) || (end<start)) return "";
    else{
        QByteArray sub = "";
        for(int k=start; k<=end; k++) sub.append(qstring[k]);
        return sub;
    }
}


//Turn the light source on and off.
bool zaberLightPositioner::Set_IRBlaster_State(bool on){
    return Send_ON_OFF_Command(on);
}

//Read the state of the light source.
bool zaberLightPositioner::Get_IRBlaster_State(){
    return Send_QUERY_Command();
}




