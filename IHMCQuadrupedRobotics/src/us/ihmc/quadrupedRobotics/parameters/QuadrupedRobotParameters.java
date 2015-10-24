package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFParameters;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.quadrupedRobotics.footstepChooser.QuadrupedControllerParameters;
import us.ihmc.quadrupedRobotics.footstepChooser.SwingTargetGeneratorParameters;

public interface QuadrupedRobotParameters
{
   public SDFRobot createSdfRobot();

   public QuadrupedJointNameMap getJointMap();

   public SDFFullRobotModel createFullRobotModel();

   public String getModelName();

   public SDFParameters getSdfParameters();

   public QuadrupedPhysicalProperties getPhysicalProperties();

   public QuadrupedControllerParameters getQuadrupedControllerParameters();

}