package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFParameters;
import us.ihmc.SdfLoader.SDFRobot;

public interface QuadrupedRobotParameters
{
   public SDFRobot createSdfRobot();

   public QuadrupedJointNameMap getJointMap();
   
   public QuadrupedJointLimits getJointLimits();

   public SDFFullRobotModel createFullRobotModel();

   public String getModelName();

   public SDFParameters getSdfParameters();

   public QuadrupedPhysicalProperties getPhysicalProperties();

   public QuadrupedControllerParameters getQuadrupedControllerParameters();
   
   public QuadrupedStandPrepParameters getQuadrupedStandPrepParameters();
   
   public QuadrupedVMCStandParameters getQuadrupedVMCStandParameters();

   public QuadrupedActuatorParameters getActuatorParameters();
}