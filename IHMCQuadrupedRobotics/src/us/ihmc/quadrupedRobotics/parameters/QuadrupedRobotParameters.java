package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFParameters;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedContactForceLimits;
import us.ihmc.quadrupedRobotics.virtualModelController.QuadrupedJointLimits;

public abstract class QuadrupedRobotParameters
{
   private final QuadrupedJointLimits jointLimits = new QuadrupedJointLimits();
   private final QuadrupedContactForceLimits contactForceLimits = new QuadrupedContactForceLimits();
   private final QuadrupedStandPrepParameters standPrepParameters = new DefaultQuadrupedStandPrepParameters();
   private final QuadrupedVirtualModelBasedStandParameters virtualModelBasedStandParameters = new DefaultQuadrupedVirtualModelBasedStandParameters();

   public abstract SDFRobot createSdfRobot();

   public abstract QuadrupedJointNameMap getJointMap();
   
   public abstract SDFFullRobotModel createFullRobotModel();

   public abstract String getModelName();

   public abstract SDFParameters getSdfParameters();

   public abstract QuadrupedPhysicalProperties getPhysicalProperties();

   public abstract QuadrupedActuatorParameters getActuatorParameters();

   public abstract QuadrupedInitialPositionParameters getQuadrupedInitialPositionParameters();

   public abstract QuadrupedControllerParameters getQuadrupedControllerParameters();
   
   public QuadrupedJointLimits getQuadrupedJointLimits()
   {
      return jointLimits;
   }
   
   public QuadrupedContactForceLimits getQuadrupedContactForceLimits()
   {
      return contactForceLimits;
   }
   
   public QuadrupedStandPrepParameters getQuadrupedStandPrepParameters()
   {
      return standPrepParameters;
   }
   
   public QuadrupedVirtualModelBasedStandParameters getQuadrupedVirtualModelBasedStandParameters()
   {
      return virtualModelBasedStandParameters;
   }
}
