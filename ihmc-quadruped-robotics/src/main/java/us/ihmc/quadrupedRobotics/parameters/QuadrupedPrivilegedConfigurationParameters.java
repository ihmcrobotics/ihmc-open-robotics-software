package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public abstract class QuadrupedPrivilegedConfigurationParameters extends JointPrivilegedConfigurationParameters
{
   public abstract double getPrivilegedConfiguration(RobotQuadrant quadrant);

   public abstract double getKneeConfigurationGain();

   public abstract double getKneeConfigurationVelocityGain();

   public abstract double getKneeWeight();
}
