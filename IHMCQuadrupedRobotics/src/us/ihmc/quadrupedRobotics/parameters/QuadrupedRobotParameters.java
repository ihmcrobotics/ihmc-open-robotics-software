package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFParameters;
import us.ihmc.SdfLoader.SDFRobot;

public interface QuadrupedRobotParameters
{
   public SDFRobot createSdfRobot();

   public QuadrupedJointNameMap getJointMap();

   public SDFFullRobotModel createFullRobotModel();

   public String getModelName();

   public SDFParameters getSdfParameters();

   public QuadrupedPhysicalProperties getPhysicalProperties();

   public QuadrupedControllerParameters getQuadrupedControllerParameters();

   public QuadrupedActuatorParameters getActuatorParameters();

}