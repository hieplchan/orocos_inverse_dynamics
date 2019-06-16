# Orocos KDL Inverse Dynamics Test
- Using Orocos KDL to calculate moment of each joint of PUMA 560
- Export Inertia Matrix, Coriolis Matrix, Gravity Matrix to .csv file
- Compare result to Matlab simulink, error is very small.


D-H table for PUMA 560
```
//             th    d       a         alpha
L(1) = Link([ 0     0       0         pi/2    0]);
L(2) = Link([ 0 	0       0.4318	  0       0]);
L(3) = Link([ 0     0.15005	0.0203    -pi/2   0]);
L(4) = Link([ 0     0.4318	0         pi/2    0]);
L(5) = Link([ 0     0       0         -pi/2   0]);
L(6) = Link([ 0     0 	    0          0      0]);

L(1).m = 0;
L(2).m = 17.4;
L(3).m = 4.8;
L(4).m = 0.82;
L(5).m = 0.34;
L(6).m = .09;
			//X Y Z
L(1).r = [ 0    0	   0 ];
L(2).r = [ -.3638  .006    .2275];
L(3).r = [ -.0203  -.0141  .070];
L(4).r = [ 0    .019    0];
L(5).r = [ 0    0	   0];
L(6).r = [ 0    0	   .032];
			//Ixx	Iyy		Izz		Ixy		Ixz  Iyz
L(1).I = [  0	 0.35	 0	 0	 0	 0];
L(2).I = [  .13	 .524	 .539	 0	 0	 0];
L(3).I = [   .066  .086	 .0125   0	 0	 0];
L(4).I = [  1.8e-3  1.3e-3  1.8e-3  0	 0	 0];
L(5).I = [  .3e-3   .4e-3   .3e-3   0	 0	 0];
L(6).I = [  .15e-3  .15e-3  .04e-3  0	 0	 0];
```
