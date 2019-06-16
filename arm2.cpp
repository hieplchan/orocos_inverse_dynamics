#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chain.hpp>

#include <iostream>
#include <iomanip>

using namespace KDL;
using namespace std;

int main()
{	

	Vector gravity(0.0, -10, 0.0);	
	
	Chain RRBotKdl = Chain();

	RigidBodyInertia inert1 = RigidBodyInertia(1, Vector(-0.5, 0, 0), RotationalInertia(1, 1, 1, 0, 0, 0));
	RigidBodyInertia inert2 = RigidBodyInertia(1, Vector(-0.5, 0, 0), RotationalInertia(1, 1, 1, 0, 0, 0));
	
	Joint joint1(Joint::RotZ);
	Frame frame1 = Frame::DH(1, 0.0, 0.0, 0.0);
	RRBotKdl.addSegment(Segment(joint1, frame1, inert1));

	Joint joint2(Joint::RotZ);
	Frame frame2 = Frame::DH(1, 0.0, 0.0, 0.0);
	RRBotKdl.addSegment(Segment(joint2, frame2, inert2));

	JntArray jointAngles = JntArray(2);
	jointAngles(0) = 0;       // Joint 1
	jointAngles(1) = 0;       // Joint 2

	JntArray jointVel = JntArray(2);
	jointVel(0) = 30;       // Joint 1
	jointVel(1) = 0;       // Joint 2


	JntArray jointAcc = JntArray(2);
	jointAcc(0) = 30;       // Joint 1
	jointAcc(1) = 0;       // Joint 2

	Wrenches jnt_wrenches;
	jnt_wrenches.push_back(Wrench()); //Cho luc vao dau canh tay 1	
	jnt_wrenches.push_back(Wrench(Vector(0, 0, 0), Vector(0, 0, 0))); //Cho luc vao dau canh tay 2	

	JntArray jointforce = JntArray(2);

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

	return 0;
}

