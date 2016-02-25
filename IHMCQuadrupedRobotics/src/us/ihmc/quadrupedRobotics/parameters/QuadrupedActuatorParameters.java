package us.ihmc.quadrupedRobotics.parameters;

import java.util.ArrayList;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.robotics.kinematics.JointLimit;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedActuatorParameters
{
   public double getLegKp();
   public double getLegKd();
   public double getLegSoftTorqueLimit();

   public double getNeckKp();
   public double getNeckKd();
   public double getNeckSoftTorqueLimit();
   
   public JointLimit getJointLimts(RobotQuadrant robotQuadrant, LegJointName legJointName);
   public ArrayList<LegJointName> getAvailableLegJoints(RobotQuadrant robotQuadrant);
}
