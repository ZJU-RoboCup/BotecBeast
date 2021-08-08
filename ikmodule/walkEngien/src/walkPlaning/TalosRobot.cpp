#include <iostream>
#include "../include/walkPlaning/TalosRobot.h"
#include "../include/walkPlaning/RobotDimensions.h"


//////RBDyn
#include "Conversions.h"
#include "Util.h"
#include "EigenUtility.h"

#include "Jacobian.h"
#include "Coriolis.h"
#include "FD.h"
#include "FK.h"
#include "FV.h"
#include "FA.h"
#include "IK.h"
#include "CoM.h"
#include "Momentum.h"
#include "ID.h"
#include "Body.h"

//////


using namespace ljnoid;
using namespace std;


TalosRobot* TalosRobot::m_UniqueInstance= new TalosRobot();   //why   but delete will fault

TalosRobot::TalosRobot()
{

	// timeStep=0.02;//s
	constructRobot();
	jointParaInit();

}


TalosRobot::~TalosRobot()
{

}



void TalosRobot::jointParaInit()
{
	// Eigen::Matrix<double,12,1> jvalue;
	// jvalue.setZero();
	// sVectorToParam(talos,jvalue);

	talosmbc.zero(talos);
	measuredmbc.zero(talos);
	expectedmbc.zero(talos);

	talosFloatmbc.zero(talos);
	measuredFloatmbc.zero(talos);

	//  talosmbc.q={{1.,0.,0.,0.,0.,0.,0.},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{}    //leftLeg 
	// ,{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{}};    //rightLeg
	// talosmbc.alpha={{0.,0.,0.,0.,0.,0.},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{}    //leftLeg 
	// ,{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{0.*TO_RADIAN},{}};    //rightLeg
	
	// // talosmbc.q={{},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{}    //leftLeg 
	// // ,{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{}};    //rightLeg
	// talosmbc.q={{},{0.*Util::TO_RADIAN},{-90.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{}    //leftLeg 
	// ,{0.*Util::TO_RADIAN},{90.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{0.*Util::TO_RADIAN},{}};    //rightLeg
	// rbd::forwardKinematics(talos, talosmbc);
	// for(int i = 0; i < talos.joints().size(); ++i)
	// {
		
	// 	// std::cout << "direction["<<i<<"] : " <<  std::endl<<  talos.joint(i).direction() << std::endl;
	// 	std::cout << "bodyPosW["<<i<<"] : " <<  std::endl<<  sva::conversions::toHomogeneous(talosmbc.bodyPosW[i]) << std::endl;
	// }
	// std::cout << "com:" <<  std::endl<<  rbd::computeCoM(talos,talosmbc).transpose() << std::endl;

	// qref=sParamToVector(talos,talosmbc.q);
	// dqref=sDofToVector(talos,talosmbc.alpha);
	// qrefOld=qref;
	// qold=qref;
	// dqrefOld=dqref;


}



void TalosRobot::constructRobot()
{
	constructTorso();
	constructLeftLeg();
	constructRightLeg();
	constructLeftArm();
	constructRightArm();


	talos=mbg.makeMultiBody("Torso",true);

	talosmbc=rbd::MultiBodyConfig(talos);
	measuredmbc=rbd::MultiBodyConfig(talos);
	expectedmbc=rbd::MultiBodyConfig(talos);


	talosFloat=mbg.makeMultiBody("Torso",rbd::Joint::Free);
	talosFloatmbc=rbd::MultiBodyConfig(talosFloat);
	measuredFloatmbc=rbd::MultiBodyConfig(talosFloat);

}

void TalosRobot::constructTorso()
{
	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

	double mass1=4.084;
	Eigen::Matrix3d linkI1;
	linkI1 << 2.564e-02*mass1,0,0,
			  0,2.477e-02*mass1,0,
			  0,0,2.467e-02*mass1;
	Eigen::Vector3d com1 =Eigen::Vector3d(0.,0.,0.);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
	sva::RBInertiad rb1(mass1,mass1*com1,I1);
	rbd::Body link1(rb1,"Torso");

	mbg.addBody(link1);

}

void TalosRobot::constructLeftLeg()
{
	
	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

	double mass1=0.1;
	Eigen::Matrix3d linkI1;
	linkI1 << 3.677e-03*mass1,0,0,
			  0,3.262e-03*mass1,0,
			  0,0,1.73e-03*mass1;
	Eigen::Vector3d com1 =Eigen::Vector3d(-0.021594032645226	,	-0.00010129064321518	,	-0.0020116567611694);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
	sva::RBInertiad rb1(mass1,mass1*com1,I1);
	rbd::Body link1(rb1,"leftLegLink1");


	double mass2=0.08;
	Eigen::Matrix3d linkI2;
	linkI2 << 4.579e-03*mass2,0,0,
			  0,4.332e-03*mass2,0,
			  0,0,2.073e-03*mass2;
	Eigen::Vector3d com2 =Eigen::Vector3d(-0.0051051788032055	,	-5.5834650993347e-05	,	-0.013482987880707);	
	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, com2,iden3d);	  
	sva::RBInertiad rb2(mass2,mass2*com2,I2);
	rbd::Body link2(rb2,"leftLegLink2");


	double mass3=0.15;
	Eigen::Matrix3d linkI3;
	linkI3 << 1.144e-02*mass3,0,0,
			  0,1.13e-02*mass3,0,
			  0,0,2.437e-03*mass3;
	Eigen::Vector3d com3 =Eigen::Vector3d(-0.0046495804563165	,	0.0029697194695473	,	-0.070015147328377);
	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, com3,iden3d);		  
	sva::RBInertiad rb3(mass3,mass3*com3,I3);
	rbd::Body link3(rb3,"leftLegLink3");


	double mass4=0.08;
	Eigen::Matrix3d linkI4;
	linkI4 << 8.708e-03*mass4,0,0,
			  0,8.421e-03*mass4,0,
			  0,0,1.72e-03*mass4;
	Eigen::Vector3d com4 =Eigen::Vector3d(0.0003126934170723	,	0.0031309314072132	,	-0.061646454036236);
	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, com4,iden3d);	  
	sva::RBInertiad rb4(mass4,mass4*com4,I4);
	rbd::Body link4(rb4,"leftLegLink4");


	double mass5=0.16;
	Eigen::Matrix3d linkI5;
	linkI5 << 6.971e-05*mass5,0,0,
			  0,5.663e-05*mass5,0,
			  0,0,5.663e-05*mass5;
	Eigen::Vector3d com5 =Eigen::Vector3d(-0.023038865998387	,	0.0033160001039505	,	0.012371957302094);	
	Eigen::Matrix3d I5=sva::inertiaToOrigin(linkI5,mass5, com5,iden3d);  
	sva::RBInertiad rb5(mass5,mass5*com5,I5);
	rbd::Body link5(rb5,"leftLegLink5");


	double mass6=0.08;
	Eigen::Matrix3d linkI6;
	linkI6 << 5.544e-03*mass6,0,0,   //in center of mass 
			  0,4.769e-03*mass6,0,
			  0,0,2.551e-03*mass6;
	Eigen::Vector3d com6 =Eigen::Vector3d(-0.0025015184655786	,	0.014091368764639	,	-0.029957683756948);
	Eigen::Matrix3d I6=sva::inertiaToOrigin(linkI6,mass6, com6,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb6(mass6,mass6*com6,I6);
	rbd::Body link6(rb6,"leftLegLink6");

	double mass7=0.0;
	Eigen::Matrix3d linkI7;
	linkI7 << 0.0,0,0,   //in center of mass 
			  0,0.0,0,
			  0,0,0.0;
	Eigen::Vector3d com7 =Eigen::Vector3d(0.0,0.0,0.0);
	Eigen::Matrix3d I7=sva::inertiaToOrigin(linkI7,mass7, com7,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb7(mass7,mass7*com7,I7);
	rbd::Body link7(rb7,"leftLegLinkSole");


	mbg.addBody(link1);
	mbg.addBody(link2);
	mbg.addBody(link3);
	mbg.addBody(link4);
	mbg.addBody(link5);
	mbg.addBody(link6);
	mbg.addBody(link7);



	rbd::Joint j1(rbd::Joint::Rev,Eigen::Vector3d(0.,0.,1.), true,"leftLegJoint1");
	rbd::Joint j2(rbd::Joint::Rev,Eigen::Vector3d(-1.,0.,0.), true,"leftLegJoint2");
	rbd::Joint j3(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"leftLegJoint3");
	rbd::Joint j4(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"leftLegJoint4");
	rbd::Joint j5(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"leftLegJoint5");
	rbd::Joint j6(rbd::Joint::Rev,Eigen::Vector3d(-1.,0.,0.), true,"leftLegJoint6");
	rbd::Joint j7(rbd::Joint::Fixed,true,"leftLegJointSole");

	mbg.addJoint(j1);
	mbg.addJoint(j2);
	mbg.addJoint(j3);
	mbg.addJoint(j4);
	mbg.addJoint(j5);
	mbg.addJoint(j6);
	mbg.addJoint(j7);




	sva::PTransformd to1(Eigen::Vector3d(Hip_X,Hip_Y,-Hip_Z));
	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("Torso",to1,"leftLegLink1",from1,"leftLegJoint1");

	sva::PTransformd to2(Eigen::Vector3d(Hip_Thigh1_X,Hip_Thigh1_Y,-Hip_Thigh1_Z));
	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink1",to2,"leftLegLink2",from2,"leftLegJoint2");

	sva::PTransformd to3(Eigen::Vector3d(Thigh1_Thigh_X,-Thigh1_Thigh_Y,-Thigh1_Thigh_Z));
	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink2",to3,"leftLegLink3",from3,"leftLegJoint3");

	sva::PTransformd to4(Eigen::Vector3d(Thigh_Knee_X,Thigh_Knee_Y,-Thigh_Knee_Z));
	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink3",to4,"leftLegLink4",from4,"leftLegJoint4");

	sva::PTransformd to5(Eigen::Vector3d(-Knee_AnkleP_X,Knee_AnkleP_Y,-Knee_AnkleP_Z));
	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink4",to5,"leftLegLink5",from5,"leftLegJoint5");

	sva::PTransformd to6(Eigen::Vector3d(AnkleP_R_X,AnkleP_R_Y,-AnkleP_R_Z));
	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink5",to6,"leftLegLink6",from6,"leftLegJoint6");

	sva::PTransformd to7(Eigen::Vector3d(Foot_X,Foot_Y,-Foot_Z));
	sva::PTransformd from7(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftLegLink6",to7,"leftLegLinkSole",from7,"leftLegJointSole");

	
	
}  




void TalosRobot::constructRightLeg()
{
	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

	double mass1=0.1;
	Eigen::Matrix3d linkI1;
	linkI1 << 3.677e-03*mass1,0,0,
			  0,3.262e-03*mass1,0,
			  0,0,1.73e-03*mass1;
	Eigen::Vector3d com1 =Eigen::Vector3d(-0.021590292453766	,	-0.00042901933193207	,	-0.0020172894001007);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
	sva::RBInertiad rb1(mass1,mass1*com1,I1);
	rbd::Body link1(rb1,"rightLegLink1");


	double mass2=0.08;
	Eigen::Matrix3d linkI2;
	linkI2 << 4.579e-03*mass2,0,0,
			  0,4.332e-03*mass2,0,
			  0,0,2.073e-03*mass2;
	Eigen::Vector3d com2 =Eigen::Vector3d(-0.0051066726446152	,	-6.3341110944748e-05	,	-0.01348876953125);	
	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, com2,iden3d);	  
	sva::RBInertiad rb2(mass2,mass2*com2,I2);
	rbd::Body link2(rb2,"rightLegLink2");


	double mass3=0.15;
	Eigen::Matrix3d linkI3;
	linkI3 << 1.144e-02*mass3,0,0,
			  0,1.13e-02*mass3,0,
			  0,0,2.437e-03*mass3;
	Eigen::Vector3d com3 =Eigen::Vector3d(-0.0045839278027415	,	-0.002898458391428	,	-0.070030435919762);
	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, com3,iden3d);		  
	sva::RBInertiad rb3(mass3,mass3*com3,I3);
	rbd::Body link3(rb3,"rightLegLink3");


	double mass4=0.08;
	Eigen::Matrix3d linkI4;
	linkI4 << 8.708e-03*mass4,0,0,
			  0,8.421e-03*mass4,0,
			  0,0,1.72e-03*mass4;
	Eigen::Vector3d com4 =Eigen::Vector3d(0.00037438794970512	,	-0.0026194229722023	,	-0.061670936644077);
	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, com4,iden3d);	  
	sva::RBInertiad rb4(mass4,mass4*com4,I4);
	rbd::Body link4(rb4,"rightLegLink4");


	double mass5=0.16;
	Eigen::Matrix3d linkI5;
	linkI5 << 6.971e-05*mass5,0,0,
			  0,5.663e-05*mass5,0,
			  0,0,5.663e-05*mass5;
	Eigen::Vector3d com5 =Eigen::Vector3d(-0.024572189897299	,	-0.003216341137886	,	0.012335173785686);	
	Eigen::Matrix3d I5=sva::inertiaToOrigin(linkI5,mass5, com5,iden3d);  
	sva::RBInertiad rb5(mass5,mass5*com5,I5);
	rbd::Body link5(rb5,"rightLegLink5");


	double mass6=0.08;
	Eigen::Matrix3d linkI6;
	linkI6 << 5.544e-03*mass6,0,0,   //in center of mass 
			  0,4.769e-03*mass6,0,
			  0,0,2.551e-03*mass6;
	Eigen::Vector3d com6 =Eigen::Vector3d(-0.0021749781444669	,	-0.013426892459393	,	-0.029985383152962);
	Eigen::Matrix3d I6=sva::inertiaToOrigin(linkI6,mass6, com6,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb6(mass6,mass6*com6,I6);
	rbd::Body link6(rb6,"rightLegLink6");

	double mass7=0.0;
	Eigen::Matrix3d linkI7;
	linkI7 << 0.0,0,0,   //in center of mass 
			  0,0.0,0,
			  0,0,0.0;
	Eigen::Vector3d com7 =Eigen::Vector3d(0.0,0.0,0.0);
	Eigen::Matrix3d I7=sva::inertiaToOrigin(linkI7,mass7, com7,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb7(mass7,mass7*com7,I7);
	rbd::Body link7(rb7,"rightLegLinkSole");


	mbg.addBody(link1);
	mbg.addBody(link2);
	mbg.addBody(link3);
	mbg.addBody(link4);
	mbg.addBody(link5);
	mbg.addBody(link6);
	mbg.addBody(link7);



	rbd::Joint j1(rbd::Joint::Rev,Eigen::Vector3d(0.,0.,1.), true,"rightLegJoint1");
	rbd::Joint j2(rbd::Joint::Rev,Eigen::Vector3d(-1.,0.,0.), true,"rightLegJoint2");
	rbd::Joint j3(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.),true,"rightLegJoint3");
	rbd::Joint j4(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"rightLegJoint4");
	rbd::Joint j5(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"rightLegJoint5");
	rbd::Joint j6(rbd::Joint::Rev,Eigen::Vector3d(-1.,0.,0.), true,"rightLegJoint6");
	rbd::Joint j7(rbd::Joint::Fixed,true,"rightLegJointSole");


	mbg.addJoint(j1);
	mbg.addJoint(j2);
	mbg.addJoint(j3);
	mbg.addJoint(j4);
	mbg.addJoint(j5);
	mbg.addJoint(j6);
	mbg.addJoint(j7);



	sva::PTransformd to1(Eigen::Vector3d(Hip_X,-Hip_Y,-Hip_Z));
	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("Torso",to1,"rightLegLink1",from1,"rightLegJoint1");

	sva::PTransformd to2(Eigen::Vector3d(Hip_Thigh1_X,-Hip_Thigh1_Y,-Hip_Thigh1_Z));
	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink1",to2,"rightLegLink2",from2,"rightLegJoint2");

	sva::PTransformd to3(Eigen::Vector3d(Thigh1_Thigh_X,Thigh1_Thigh_Y,-Thigh1_Thigh_Z));
	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink2",to3,"rightLegLink3",from3,"rightLegJoint3");

	sva::PTransformd to4(Eigen::Vector3d(Thigh_Knee_X,Thigh_Knee_Y,-Thigh_Knee_Z));
	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink3",to4,"rightLegLink4",from4,"rightLegJoint4");

	sva::PTransformd to5(Eigen::Vector3d(-Knee_AnkleP_X,Knee_AnkleP_Y,-Knee_AnkleP_Z));
	sva::PTransformd from5(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink4",to5,"rightLegLink5",from5,"rightLegJoint5");

	sva::PTransformd to6(Eigen::Vector3d(AnkleP_R_X,-AnkleP_R_Y,-AnkleP_R_Z));
	sva::PTransformd from6(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink5",to6,"rightLegLink6",from6,"rightLegJoint6");

	sva::PTransformd to7(Eigen::Vector3d(Foot_X,-Foot_Y,-Foot_Z));
	sva::PTransformd from7(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightLegLink6",to7,"rightLegLinkSole",from7,"rightLegJointSole");



}
void TalosRobot::constructLeftArm()
{

	Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

	double mass1=4.000e-02;
	Eigen::Matrix3d linkI1;
	linkI1 << 2.945e-04*mass1,0,0,
			  0,2.213e-04*mass1,0,
			  0,0,1.527e-04*mass1;
	Eigen::Vector3d com1 =Eigen::Vector3d(+1.156e-03,	+3.580e-04,	+6.187e-04);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
	sva::RBInertiad rb1(mass1,mass1*com1,I1);
	rbd::Body link1(rb1,"leftArmLink1");


	double mass2=1.600e-01;
	Eigen::Matrix3d linkI2;
	linkI2 << 1.023e-03*mass2,0,0,
			  0,1.023e-03*mass2,0,
			  0,0,2.040e-04*mass2;
	Eigen::Vector3d com2 =Eigen::Vector3d(-8.745e-06,	-2.277e-05,	-2.692e-07);	
	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, com2,iden3d);	  
	sva::RBInertiad rb2(mass2,mass2*com2,I2);
	rbd::Body link2(rb2,"leftArmLink2");


	double mass3=1.500e-01;
	Eigen::Matrix3d linkI3;
	linkI3 << 1.386e-03*mass3,0,0,
			  0,1.297e-03*mass3,0,
			  0,0,2.875e-04*mass3;
	Eigen::Vector3d com3 =Eigen::Vector3d(+6.766e-04,	+2.013e-05,	-3.317e-03);
	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, com3,iden3d);		  
	sva::RBInertiad rb3(mass3,mass3*com3,I3);
	rbd::Body link3(rb3,"leftArmLink3");



	double mass4=0.0;
	Eigen::Matrix3d linkI4;
	linkI4 << 0.0,0,0,   //in center of mass 
			  0,0.0,0,
			  0,0,0.0;
	Eigen::Vector3d com4 =Eigen::Vector3d(0.0,0.0,0.0);
	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, com4,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb4(mass4,mass4*com4,I4);
	rbd::Body link4(rb4,"leftArmLinkSole");

	mbg.addBody(link1);
	mbg.addBody(link2);
	mbg.addBody(link3);
	mbg.addBody(link4);



	rbd::Joint j1(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"leftArmJoint1");
	rbd::Joint j2(rbd::Joint::Rev,Eigen::Vector3d(1.,0.,0.), true,"leftArmJoint2");
	rbd::Joint j3(rbd::Joint::Rev,Eigen::Vector3d(1.,0.,0.), true,"leftArmJoint3");
	rbd::Joint j4(rbd::Joint::Fixed,true,"leftArmJointSole");

	mbg.addJoint(j1);
	mbg.addJoint(j2);
	mbg.addJoint(j3);
	mbg.addJoint(j4);



	sva::PTransformd to1(Eigen::Vector3d(Shoulder2_X,Shoulder2_Y,Shoulder2_Z));
	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("Torso",to1,"leftArmLink1",from1,"leftArmJoint1");

	sva::PTransformd to2(Eigen::Vector3d(Shoulder1_X,Shoulder1_Y,-Shoulder1_Z));
	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftArmLink1",to2,"leftArmLink2",from2,"leftArmJoint2");

	sva::PTransformd to3(Eigen::Vector3d(-Elbow1_X,Elbow1_Y,Elbow1_Z));
	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftArmLink2",to3,"leftArmLink3",from3,"leftArmJoint3");

	sva::PTransformd to4(Eigen::Vector3d(-Wrist1_X,Wrist1_Y,Wrist1_Z));
	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("leftArmLink3",to4,"leftArmLinkSole",from4,"leftArmJointSole");

	
}

void TalosRobot::constructRightArm()
{
    Eigen::Matrix3d iden3d=Eigen::Matrix3d::Identity();

	double mass1=4.000e-02;
	Eigen::Matrix3d linkI1;
	linkI1 << 2.945e-04*mass1,0,0,
			  0,2.213e-04*mass1,0,
			  0,0,1.527e-04*mass1;
	Eigen::Vector3d com1 =Eigen::Vector3d(+1.156e-03,	+3.580e-04,	+6.187e-04);	
	Eigen::Matrix3d I1=sva::inertiaToOrigin(linkI1,mass1, com1,iden3d);	  
	sva::RBInertiad rb1(mass1,mass1*com1,I1);
	rbd::Body link1(rb1,"rightArmLink1");


	double mass2=1.600e-01;
	Eigen::Matrix3d linkI2;
	linkI2 << 1.023e-03*mass2,0,0,
			  0,1.023e-03*mass2,0,
			  0,0,2.040e-04*mass2;
	Eigen::Vector3d com2 =Eigen::Vector3d(-8.745e-06,	-2.277e-05,	-2.692e-07);	
	Eigen::Matrix3d I2=sva::inertiaToOrigin(linkI2,mass2, com2,iden3d);	  
	sva::RBInertiad rb2(mass2,mass2*com2,I2);
	rbd::Body link2(rb2,"rightArmLink2");


	double mass3=1.500e-01;
	Eigen::Matrix3d linkI3;
	linkI3 << 1.386e-03*mass3,0,0,
			  0,1.297e-03*mass3,0,
			  0,0,2.875e-04*mass3;
	Eigen::Vector3d com3 =Eigen::Vector3d(+6.766e-04,	+2.013e-05,	-3.317e-03);
	Eigen::Matrix3d I3=sva::inertiaToOrigin(linkI3,mass3, com3,iden3d);		  
	sva::RBInertiad rb3(mass3,mass3*com3,I3);
	rbd::Body link3(rb3,"rightArmLink3");



	double mass4=0.0;
	Eigen::Matrix3d linkI4;
	linkI4 << 0.0,0,0,   //in center of mass 
			  0,0.0,0,
			  0,0,0.0;
	Eigen::Vector3d com4 =Eigen::Vector3d(0.0,0.0,0.0);
	Eigen::Matrix3d I4=sva::inertiaToOrigin(linkI4,mass4, com4,iden3d);	//in body frame orgin	  
	sva::RBInertiad rb4(mass4,mass4*com4,I4);
	rbd::Body link4(rb4,"rightArmLinkSole");

	mbg.addBody(link1);
	mbg.addBody(link2);
	mbg.addBody(link3);
	mbg.addBody(link4);



	rbd::Joint j1(rbd::Joint::Rev,Eigen::Vector3d(0.,1.,0.), true,"rightArmJoint1");
	rbd::Joint j2(rbd::Joint::Rev,Eigen::Vector3d(1.,0.,0.), true,"rightArmJoint2");
	rbd::Joint j3(rbd::Joint::Rev,Eigen::Vector3d(1.,0.,0.), true,"rightArmJoint3");
	rbd::Joint j4(rbd::Joint::Fixed,true,"rightArmJointSole");

	mbg.addJoint(j1);
	mbg.addJoint(j2);
	mbg.addJoint(j3);
	mbg.addJoint(j4);



	sva::PTransformd to1(Eigen::Vector3d(Shoulder2_X,-Shoulder2_Y,Shoulder2_Z));
	sva::PTransformd from1(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("Torso",to1,"rightArmLink1",from1,"rightArmJoint1");

	sva::PTransformd to2(Eigen::Vector3d(Shoulder1_X,-Shoulder1_Y,-Shoulder1_Z));
	sva::PTransformd from2(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightArmLink1",to2,"rightArmLink2",from2,"rightArmJoint2");

	sva::PTransformd to3(Eigen::Vector3d(-Elbow1_X,-Elbow1_Y,Elbow1_Z));
	sva::PTransformd from3(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightArmLink2",to3,"rightArmLink3",from3,"rightArmJoint3");

	sva::PTransformd to4(Eigen::Vector3d(-Wrist1_X,-Wrist1_Y,Wrist1_Z));
	sva::PTransformd from4(Eigen::Vector3d(0., 0., 0.));
	mbg.linkBodies("rightArmLink3",to4,"rightArmLinkSole",from4,"rightArmJointSole");

}
