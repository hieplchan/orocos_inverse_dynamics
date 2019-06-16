#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chain.hpp>

#include <iostream>
#include <iomanip>

using namespace KDL;
using namespace std;

int main()
{	

	Vector gravity(0, 0, -10);	
	
	Chain RRBotKdl = Chain();

	RigidBodyInertia inert1 = RigidBodyInertia(0, Vector(0, 0, 0), RotationalInertia(0, 0.35, 0, 0, 0, 0));
	RigidBodyInertia inert2 = RigidBodyInertia(17.4, Vector(-0.3638, 0.006, 0.2275), RotationalInertia(0.13, 0.524, 0.539, 0, 0, 0));
	RigidBodyInertia inert3 = RigidBodyInertia(4.8, Vector(-0.0203, -0.0141, 0.070), RotationalInertia(0.066, 0.086, 0.0125, 0, 0, 0));
	RigidBodyInertia inert4 = RigidBodyInertia(0.82, Vector(0, 0.019, 0), RotationalInertia(10.8e-3, 10.3e-3, 10.8e-3, 0, 0, 0));
	RigidBodyInertia inert5 = RigidBodyInertia(0.34, Vector(0, 0, 0), RotationalInertia(0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0));
	RigidBodyInertia inert6 = RigidBodyInertia(0.09, Vector(0, 0, 0.032), RotationalInertia(0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0));
	

	Frame frame1 = Frame::DH(0, M_PI/2, 0, 0);
	Frame frame2 = Frame::DH(0.4318, 0, 0, 0);
	Frame frame3 = Frame::DH(0.0203, -M_PI/2, 0.15005, 0);
	Frame frame4 = Frame::DH(0, M_PI/2, 0.4318, 0);
	Frame frame5 = Frame::DH(0, -M_PI/2, 0, 0);
	Frame frame6 = Frame::DH(0, 0, 0, 0);


	Joint joint1(Joint::RotZ);
	Joint joint2(Joint::RotZ);
	Joint joint3(Joint::RotZ);
	Joint joint4(Joint::RotZ);
	Joint joint5(Joint::RotZ);
	Joint joint6(Joint::RotZ);


	RRBotKdl.addSegment(Segment(joint1, frame1, inert1));
	RRBotKdl.addSegment(Segment(joint2, frame2, inert2));
	RRBotKdl.addSegment(Segment(joint3, frame3, inert3));
	RRBotKdl.addSegment(Segment(joint4, frame4, inert4));
	RRBotKdl.addSegment(Segment(joint5, frame5, inert5));
	RRBotKdl.addSegment(Segment(joint6, frame6, inert6));

	JntArray jointAngles = JntArray(6);
	jointAngles(0) = 0;       // Joint 1
	jointAngles(1) = 0;       // Joint 2
	jointAngles(2) = 0;       // Joint 3
	jointAngles(3) = 0;       // Joint 4
	jointAngles(4) = 0;       // Joint 5
	jointAngles(5) = 0;       // Joint 6


	JntArray jointVel = JntArray(6);
	jointVel(0) = 0;    
	jointVel(1) = 0;       
	jointVel(2) = 0;       
	jointVel(3) = 0;       
	jointVel(4) = 0;       
	jointVel(5) = 0;       
   

	JntArray jointAcc = JntArray(6);
	jointAcc(0) = 0;
	jointAcc(1) = 0;
	jointAcc(2) = 0;
	jointAcc(3) = 0;
	jointAcc(4) = 0;
	jointAcc(5) = 0	;

	Wrenches jnt_wrenches;
	//jnt_wrenches.push_back(Wrench(Vector(0, 0, -10), Vector(0, 0, 0)));
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench());


	JntArray jointforce = JntArray(6);

	//Forward Kinematic

	ChainFkSolverPos_recursive FKSolver = ChainFkSolverPos_recursive(RRBotKdl);
	Frame eeFrame;
	FKSolver.JntToCart(jointAngles, eeFrame);

	//Inverse Dynamic

	ChainIdSolver_RNE gcSolver = ChainIdSolver_RNE(RRBotKdl, gravity);
	int ret = gcSolver.CartToJnt(jointAngles, jointVel, jointAcc, jnt_wrenches, jointforce);
	if (ret < 0) cout << "ERROR" << endl;


	// Print the frame
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++) {
			double a = eeFrame(i, j);
			if (a < 0.0001 && a > -0.001) {
				a = 0.0;
			}
			std::cout << std::setprecision(4) << a << "\t\t";
		}
		std::cout << std::endl;
	}

	cout << jointforce(0) << endl;
	cout << jointforce(1) << endl;
	cout << jointforce(2) << endl;
	cout << jointforce(3) << endl;
	cout << jointforce(4) << endl;
	cout << jointforce(5) << endl;

	return 0;
}

