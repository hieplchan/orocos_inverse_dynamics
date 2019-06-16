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

	RigidBodyInertia inert1 = RigidBodyInertia(0, Vector(-0.5, 0, 0), RotationalInertia(0, 0, 1/3, 0, 0, 0));
	
	Joint joint1(Joint::RotZ);
	Frame frame1 = Frame::DH(1, 0.0, 0.0, 0.0);
	RRBotKdl.addSegment(Segment(joint1, frame1, inert1));

	JntArray jointAngles = JntArray(1);
	jointAngles(0) = 0;       // Joint 1

	JntArray jointVel = JntArray(1);
	jointVel(0) = 0;       // Joint 1

	JntArray jointAcc = JntArray(1);
	jointAcc(0) = 1;       // Joint 1

	Wrenches jnt_wrenches;
	//jnt_wrenches.push_back(Wrench());
	jnt_wrenches.push_back(Wrench(Vector(0, -10, 0), Vector(0, 0, 0)));

	JntArray jointforce = JntArray(1);

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

	return 0;
}

