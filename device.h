#ifndef DEVICE_H
#define DEVICE_H

#pragma once
#include <string>
#include <thread>
#include <mutex>
#include <future>
#include <QVector>
//#include <zmq.hpp>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextStream>
//#include "ASL.pb.h"
//#include <google/protobuf/text_format.h>

//Device specific libraries
#include <zaber/motion/ascii.h>
//#endif // DEVICE_H
using namespace std;
using namespace chrono;
using namespace std::this_thread;     // sleep_for, sleep_until
//using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;
using namespace zaber::motion;
using namespace zaber::motion::ascii;


/***********************************************************************************
 *
 * Device enumeration declarations
 */

//NOTE: Changes to this enum type should be reflected in the
// device.Get_Device_Type_As_String() function.
enum devicetype {
    no_device_type=-1,
    camera=0,
    motion_control_stage=1,
    calibration_source=2
};

//NOTE: Changes to this enum type should be reflected in the
// device.Get_Device_Subtype_As_String() function.
enum devicesubtype{

    //camera sub types
    no_device_subtype=-1,
    visible_mono=0,
    visible_color=1,
    visible_mono_dot_polarimeter=2,
    visible_mono_microgrid_polarimeter=3,
    visible_color_microgrid_polarimeter=4,

    //motion control stage sub types
    rotary_stage=20,
    translation_stage=21,
    robotic_arm=22,
    rotary_stepper_stages=23,

    //calibration source sub types
    integrating_sphere=40,
    blackbody=41
};






/***********************************************************************************
 * device Class
 *
 * This superclass defines a base class that all specialized device classes will
 * inherit from.
 */
class device {
protected:
    string device_name;
    string device_manufacturer;
    string device_model;
    enum devicetype device_type;
    enum devicesubtype device_subtype;
    bool device_detected=false;
    bool device_initialized=false;
    bool device_busy=false;
    unsigned int number_of_detection_tries = 20;
    double redetection_wait_time = 1; //second

    bool console_output = false;
public:
    device();

    //Set and Get Functions
    void Set_Device_Name(string name) {device_name = name;}
    void Set_Device_Manufacturer(string manu) {device_manufacturer = manu;}
    void Set_Device_Model(string model) {device_model = model;}
    void Set_Device_Type(devicetype type) {device_type = type;}
    void Set_Device_Subtype(devicesubtype subtype) {device_subtype = subtype;}
    void Set_Number_Of_Detection_Tries(unsigned int num_tries) {number_of_detection_tries = num_tries;}
    void Set_Redetection_Wait_Time(double wait_time) {redetection_wait_time = wait_time;}

    string Get_Device_Name() {return device_name;}
    string Get_Device_Manufacturer() {return device_manufacturer;}
    string Get_Device_Model() {return device_model;}
    devicetype Get_Device_Type() {return device_type;}    
    devicesubtype Get_Device_Subtype() {return device_subtype;}
    string Get_Device_Type_As_String();
    string Get_Device_Subtype_As_String();
    unsigned int Get_Number_Of_Detection_Tries() {return number_of_detection_tries;}
    double Get_Redetection_Wait_Time() {return redetection_wait_time;}
    bool Is_Device_Detected(){ return device_detected;}
    bool Is_Device_Initialized(){ return device_initialized;}

    //Other Useful Functions
    string Get_Timestamp(bool filename_safe);

    //==============>Functions that should be overloaded in inherited devices<==
    virtual void Detect_Device();
    virtual void Initialize_Device();
    virtual bool Is_Device_Busy(){return device_busy;}

};


/***********************************************************************************
 * zaberLightPositioner Class
 *
 * This is a device class for controlling the ASL light source positioning system.
 * It is based upon both a Zaber high precision rotation stage and stepper motor.
 * It contains all relevant functionality for control and is based upon Zaber's
 * motion controll ASCII library. This class inherits from the base device class.
 *
 * Angular limitations are imposed that restrict the stage to the range of [-X, X]
 * degrees to ensure it does not interfere with the robotic arm. As such the rotary
 * stage is also not permitted to freely rotate at a constant rate.
 *
 */
class zaberLightPositioner : public device {

private:
    //Zaber serial connection and device variables
    string zaber_serial_port_name;
    Connection connection;
    vector<Device> deviceList;
    Device *rotation_stage_device;
    Axis *rotation_stage_axis;
    Device *stepper_motor_device;
    Axis *stepper_motor_axis;

    //IRBlaster serial connection variables
    string irblaster_serial_port_name;
    QSerialPort irblaster_serial_port;
    bool irblaster_serial_port_open=false;

    //Maximum/Minimum Absolute Az/El Ranges (set by system constraints)
    //These are not settable by the user
    float max_abs_azimuth = 180.0;
    float min_abs_azimuth = -180.0;
    float max_abs_elevation = 90.0;
    float min_abs_elevation = 0.0;

    //Maximum/Minimum Constrained Az/El Ranges (to avoid collisions with other devices)
    //These are settable by the user and are restricted within the absolute Az/El ranges
    float max_con_azimuth = 175.0;
    float min_con_azimuth = -175.0;
    float max_con_elevation = 85.0;
    float min_con_elevation = 5.0;

    //Conversion factors for converting between stepper motor units and elevation degrees
    float stepper_motor_units_to_elevation_degrees = 10.0;

    //Elevation calibration procedure to determine conversion factor?

    //Serial Port Commands
    bool Send_ON_OFF_Command(bool on);
    bool Send_QUERY_Command();

    //Helper functions
    QByteArray getSubstring(const QByteArray qstring,size_t start, size_t end);

public:
    zaberLightPositioner();

    //Serial Port Functions
    bool Detect_Serial_Ports();
    void Set_Zaber_Serial_Port(string serialPort) { zaber_serial_port_name = serialPort;}
    void Set_IRBlaster_Serial_Port(string serialPort) { irblaster_serial_port_name = serialPort;}
    string Get_IRBlaster_Serial_Port_Name(){return irblaster_serial_port_name;}
    string Get_Zaber_Serial_Port_Name(){return zaber_serial_port_name;}
    bool Is_IRBlaster_Serial_Port_Open(){return irblaster_serial_port_open;}

    //Rotation Stage Commands
    bool Home_Rotation_Stage();
    bool Stop_Rotation_Stage();
    bool Set_Rotation_Stage_Angular_Position(double degrees);
    bool Set_Rotation_Stage_Relative_Angular_Position(double degrees);
    bool Set_Rotation_Stage_Constrained_MaxMin_Azimuth(double max, double min);

    double Get_Rotation_Stage_Angular_Position();
    double Get_Rotation_Stage_Maximum_Angular_Velocity();
    bool Get_Rotation_Stage_Constrained_Max_Azimuth(){return max_con_azimuth;}
    bool Get_Rotation_Stage_Constrained_Min_Azimuth(){return min_con_azimuth;}
    bool Get_Rotation_Stage_Absolute_Max_Azimuth(){return max_abs_azimuth;}
    bool Get_Rotation_Stage_Absolute_Min_Azimuth(){return min_abs_azimuth;}

    //Stepper Motor Commands
    bool Home_Stepper_Motor();
    bool Stop_Stepper_Motor();
    bool Set_Stepper_Motor_Angular_Position(double degrees);
    bool Set_Stepper_Motor_Relative_Angular_Position(double degrees);
    bool Set_Stepper_Motor_Constrained_MaxMin_Elevation(double max, double min);

    double Get_Stepper_Motor_Angular_Position();
    double Get_Stepper_Motor_Maximum_Angular_Velocity();
    double Get_Stepper_Motor_Angular_Conversion_Factor(){return stepper_motor_units_to_elevation_degrees;}
    double Get_Stepper_Motor_Constrained_Max_Elevation(){return max_con_elevation;}
    double Get_Stepper_Motor_Constrained_Min_Elevation(){return min_con_elevation;}
    double Get_Stepper_Motor_Absolute_Max_Elevation(){return max_abs_elevation;}
    double Get_Stepper_Motor_Absolute_Min_Elevation(){return min_abs_elevation;}

    //Colimated Light Source Commands
    bool Set_IRBlaster_State(bool on);
    bool Get_IRBlaster_State();

    //Overloaded functions
    void Detect_Device() override;
    void Initialize_Device() override;
    bool Is_Device_Busy() override;

};


#endif // DEVICE_H
