/*
 * Headerfile for jaco2 SDK bindings
 * Created on: Apr 12, 2023
 * Author: Daniel Dharampal
 */ 

#include "KinovaTypes.h"
#include <iostream>
#include <pybind11/pybind11.h>
#ifdef __linux__ 
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <stdio.h>
#include <unistd.h>
#elif _WIN32
#include <Windows.h>
#include "CommunicationLayer.h"
#include "CommandLayer.h"
#include <conio.h>
#include <iostream>
#endif

using namespace std;

//A handle to the API.
#ifdef __linux__ 
void * commandLayer_handle;
#elif _WIN32
HINSTANCE commandLayer_handle;
#endif

//Function pointers to the functions we need
int(*initAPI)();
int(*closeAPI)();
int(*initFingers)();

int(*sendBasicTrajectory)(TrajectoryPoint command);
int(*moveHome)();

int(*getDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*setActiveDevice)(KinovaDevice device);
int(*getGeneralInformations)(GeneralInformations &Response);

int initRobot(){
	int programResult = 0;

	#ifdef __linux__ 
	//We load the API
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library
	initAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	closeAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
    initFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");

    sendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	moveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");

	getDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	setActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
    getGeneralInformations = (int (*)(GeneralInformations &info)) dlsym(commandLayer_handle,"GetGeneralInformations");
	#elif _WIN32
	//We load the API.
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//We load the functions from the library
	initAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	closeAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
    initFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");

	sendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	moveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");

    getDevices = (int(*)(KinovaDevice[MAX_KINOVA_DEVICE], int&)) GetProcAddress(commandLayer_handle, "GetDevices");
	setActiveDevice = (int(*)(KinovaDevice)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
    getGeneralInformations = (int(*)(GeneralInformations &info)) GetProcAddress(commandLayer_handle, "GetGeneralInformations");
	#endif

	int result = 0;
	//Verify that all functions has been loaded correctly
	if ((initAPI == NULL) || (closeAPI == NULL) || (initFingers == NULL) 
        || (sendBasicTrajectory == NULL) || (moveHome == NULL)
        || (getDevices == NULL) || (setActiveDevice == NULL) || (getGeneralInformations == NULL))

	{
		cout << "Error during initialization" << endl;
		programResult = 0;
	}
	else
	{
		cout << "Initialization complete" << endl;

		result = (*initAPI)();

		cout << "Initialization's result :" << result << endl;

		KinovaDevice list[MAX_KINOVA_DEVICE];

		int devicesCount = getDevices(list, result);

		for (int i = 0; i < devicesCount; i++)
		{
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;
			//Setting the current device as the active device.
			setActiveDevice(list[i]);            
		}
	}
	return result;	
}

int closeRobot(){
	int programResult = 0;

	cout << "Closing Api" << endl;
	int result = (*closeAPI)();
	cout << "Termination result :" << result << endl;
	programResult = 1;

	#ifdef __linux__ 
	dlclose(commandLayer_handle);
	#elif _WIN32
	FreeLibrary(commandLayer_handle);
	#endif

	return programResult;
}


int myMoveHome(){
	moveHome();
	return 1;
}

int myInitFingers(){
	initFingers();
	return 1;
}

int mySendBasicTrajectory(TrajectoryPoint point){
	sendBasicTrajectory(point);
	return 1;
}

GeneralInformations myGetGeneralInformations(){
	GeneralInformations data;
	getGeneralInformations(data);
	return data;
}

TrajectoryPoint mySetTypeVelocity(TrajectoryPoint pointToSend){
	pointToSend.Position.Type = ANGULAR_VELOCITY;
	return pointToSend;
}

TrajectoryPoint mySetTypePosition(TrajectoryPoint pointToSend){
	pointToSend.Position.Type = ANGULAR_POSITION;
	return pointToSend;
}

PYBIND11_MODULE(jaco2, mod) {
	namespace py = pybind11;
    mod.def("init_robot", &initRobot, "Initialize robot");
	mod.def("close_robot", &closeRobot, "Close robot connection");
	mod.def("init_fingers", &myInitFingers, "Initialize fingers");
    mod.def("send_basic_trajectory", &mySendBasicTrajectory, "Send a trajectory with send_basic_trajectory(TrajectoryPoint)");
    mod.def("move_home", &myMoveHome, "Move to robot home position");
	mod.def("set_type_velocity", &mySetTypeVelocity, "Set movement type to velocity");
	mod.def("set_type_position", &mySetTypePosition, "Set movement type to position");
    mod.def("get_general_informations", &myGetGeneralInformations, "Get informations about the robot with get_general_informations(GeneralInformations), then the GeneralInformations object to read the data");

	py::class_<TrajectoryPoint>(mod, "TrajectoryPoint")
		.def(py::init<>())
		.def("init", &TrajectoryPoint::InitStruct)
		.def_property("actuator1",
					[](const TrajectoryPoint& tp) { return tp.Position.Actuators.Actuator1; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Actuators.Actuator1 = val; })
		.def_property("actuator2",
					[](const TrajectoryPoint& tp) { return tp.Position.Actuators.Actuator2; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Actuators.Actuator2 = val; })
		.def_property("actuator3",
					[](const TrajectoryPoint& tp) { return tp.Position.Actuators.Actuator3; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Actuators.Actuator3 = val; })
		.def_property("actuator4",
					[](const TrajectoryPoint& tp) { return tp.Position.Actuators.Actuator4; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Actuators.Actuator4 = val; })
		.def_property("actuator5",
					[](const TrajectoryPoint& tp) { return tp.Position.Actuators.Actuator5; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Actuators.Actuator5 = val; })
		.def_property("actuator6",
					[](const TrajectoryPoint& tp) { return tp.Position.Actuators.Actuator6; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Actuators.Actuator6 = val; })
		.def_property("finger1",
					[](const TrajectoryPoint& tp) { return tp.Position.Fingers.Finger1; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Fingers.Finger1 = val; })
		.def_property("finger2",
					[](const TrajectoryPoint& tp) { return tp.Position.Fingers.Finger2; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Fingers.Finger2 = val; })
		.def_property("finger3",
					[](const TrajectoryPoint& tp) { return tp.Position.Fingers.Finger3; },
					[](TrajectoryPoint& tp, float val) { tp.Position.Fingers.Finger3 = val; });

	
	py::class_<GeneralInformations>(mod, "GeneralInformations")
    	.def(py::init<>())
		.def_property("cartesian_x", 
			[](const GeneralInformations &gi) { return gi.Position.CartesianPosition.X; },
			[](GeneralInformations &gi, float value) { gi.Position.CartesianPosition.X = value; },
			"X coordinate of the end-effector in cartesian space")
		.def_property("cartesian_y", 
			[](const GeneralInformations &gi) { return gi.Position.CartesianPosition.Y; },
			[](GeneralInformations &gi, float value) { gi.Position.CartesianPosition.Y = value; },
			"Y coordinate of the end-effector in cartesian space")
		.def_property("cartesian_z", 
			[](const GeneralInformations &gi) { return gi.Position.CartesianPosition.Z; },
			[](GeneralInformations &gi, float value) { gi.Position.CartesianPosition.Z = value; },
			"Z coordinate of the end-effector in cartesian space")
		.def_property("cartesian_theta_x", 
			[](const GeneralInformations &gi) { return gi.Position.CartesianPosition.ThetaX; },
			[](GeneralInformations &gi, float value) { gi.Position.CartesianPosition.ThetaX = value; },
			"Orientation of the end-effector around the X-axis in radians")
		.def_property("cartesian_theta_y", 
			[](const GeneralInformations &gi) { return gi.Position.CartesianPosition.ThetaY; },
			[](GeneralInformations &gi, float value) { gi.Position.CartesianPosition.ThetaY = value; },
			"Orientation of the end-effector around the Y-axis in radians")
		.def_property("cartesian_theta_z", 
			[](const GeneralInformations &gi) { return gi.Position.CartesianPosition.ThetaZ; },
			[](GeneralInformations &gi, float value) { gi.Position.CartesianPosition.ThetaZ = value; },
			"Orientation of the end-effector around the Z-axis in radians")
		
		.def_property("actuator1",
					[](const  GeneralInformations &gi) { return gi.Position.Actuators.Actuator1; },
					[]( GeneralInformations &gi, float val) { gi.Position.Actuators.Actuator1 = val; },
					"Position of actuator 1")
		.def_property("actuator2",
					[](const  GeneralInformations &gi) { return gi.Position.Actuators.Actuator2; },
					[]( GeneralInformations &gi, float val) { gi.Position.Actuators.Actuator2 = val; },
					"Position of actuator 2")
		.def_property("actuator3",
					[](const  GeneralInformations &gi) { return gi.Position.Actuators.Actuator3; },
					[]( GeneralInformations &gi, float val) { gi.Position.Actuators.Actuator3 = val; },
					"Position of actuator 3")
		.def_property("actuator4",
					[](const  GeneralInformations &gi) { return gi.Position.Actuators.Actuator4; },
					[]( GeneralInformations &gi, float val) { gi.Position.Actuators.Actuator4 = val; },
					"Position of actuator 4")
		.def_property("actuator5",
					[](const  GeneralInformations &gi) { return gi.Position.Actuators.Actuator5; },
					[]( GeneralInformations &gi, float val) { gi.Position.Actuators.Actuator5 = val; },
					"Position of actuator 5")
		.def_property("actuator6",
					[](const  GeneralInformations &gi) { return gi.Position.Actuators.Actuator6; },
					[]( GeneralInformations &gi, float val) { gi.Position.Actuators.Actuator6 = val; },
					"Position of actuator 6")
		.def_property("finger1",
					[](const  GeneralInformations &gi) { return gi.Position.Fingers.Finger1; },
					[]( GeneralInformations &gi, float val) { gi.Position.Fingers.Finger1 = val; },
					"Position of finger 1")
		.def_property("finger2",
					[](const  GeneralInformations &gi) { return gi.Position.Fingers.Finger2; },
					[]( GeneralInformations &gi, float val) { gi.Position.Fingers.Finger2 = val; },
					"Position of finger 2")
		.def_property("finger3",
					[](const  GeneralInformations &gi) { return gi.Position.Fingers.Finger3; },
					[]( GeneralInformations &gi, float val) { gi.Position.Fingers.Finger3 = val; },
					"Position of finger 3")

		.def_property("force_actuator1",
					[](const  GeneralInformations &gi) { return gi.Force.Actuators.Actuator1; },
					[]( GeneralInformations &gi, float val) { gi.Force.Actuators.Actuator1 = val; },
					"Force of actuator 1")
		.def_property("force_actuator2",
					[](const  GeneralInformations &gi) { return gi.Force.Actuators.Actuator2; },
					[]( GeneralInformations &gi, float val) { gi.Force.Actuators.Actuator2 = val; },
					"Force of actuator 2")
		.def_property("force_actuator3",
					[](const  GeneralInformations &gi) { return gi.Force.Actuators.Actuator3; },
					[]( GeneralInformations &gi, float val) { gi.Force.Actuators.Actuator3 = val; },
					"Force of actuator 3")
		.def_property("force_actuator4",
					[](const  GeneralInformations &gi) { return gi.Force.Actuators.Actuator4; },
					[]( GeneralInformations &gi, float val) { gi.Force.Actuators.Actuator4 = val; },
					"Force of actuator 4")
		.def_property("force_actuator5",
					[](const  GeneralInformations &gi) { return gi.Force.Actuators.Actuator5; },
					[]( GeneralInformations &gi, float val) { gi.Force.Actuators.Actuator5 = val; },
					"Force of actuator 5")
		.def_property("force_actuator6",
					[](const  GeneralInformations &gi) { return gi.Force.Actuators.Actuator6; },
					[]( GeneralInformations &gi, float val) { gi.Force.Actuators.Actuator6 = val; },
					"Force of actuator 6")	
		.def_property_readonly("temperature1",
					[](const GeneralInformations& gi) { return gi.ActuatorsTemperatures[0]; },
					"Temperature of actuator 1")
		.def_property_readonly("temperature2",
					[](const GeneralInformations& gi) { return gi.ActuatorsTemperatures[1]; },
					"Temperature of actuator 2")
		.def_property_readonly("temperature3",
					[](const GeneralInformations& gi) { return gi.ActuatorsTemperatures[2]; },
					"Temperature of actuator 3")
		.def_property_readonly("temperature4",
					[](const GeneralInformations& gi) { return gi.ActuatorsTemperatures[3]; },
					"Temperature of actuator 4")
		.def_property_readonly("temperature5",
					[](const GeneralInformations& gi) { return gi.ActuatorsTemperatures[4]; },
					"Temperature of actuator 5")
		.def_property_readonly("temperature6",
					[](const GeneralInformations& gi) { return gi.ActuatorsTemperatures[5]; },
					"Temperature of actuator 6")
		.def_readwrite("power", &GeneralInformations::Power, "Power consumption")
		.def_readwrite("controller", &GeneralInformations::Controller, "The robot controller in use");
}

