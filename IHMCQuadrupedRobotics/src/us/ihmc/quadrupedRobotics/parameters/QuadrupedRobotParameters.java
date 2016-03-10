package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFFullQuadrupedRobotModel;
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
   private final QuadrupedContactPointParameters contactPointParameters = new QuadrupedContactPointParameters();

   public abstract SDFRobot createSdfRobot();

   public abstract QuadrupedJointNameMap getJointMap();
   
   public abstract SDFFullQuadrupedRobotModel createFullRobotModel();

   public abstract String getModelName();

   public abstract SDFParameters getSdfParameters();

   public abstract QuadrupedPhysicalProperties getPhysicalProperties();

   public abstract QuadrupedActuatorParameters getActuatorParameters();

   public abstract QuadrupedInitialPositionParameters getQuadrupedInitialPositionParameters();

   public abstract QuadrupedPositionBasedCrawlControllerParameters getQuadrupedPositionBasedCrawlControllerParameters();
   
   public abstract QuadrupedSensorInformation getQuadrupedSensorInformation();

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
   
   public QuadrupedContactPointParameters getQuadrupedContactPointParameters()
   {
      return contactPointParameters;
   }

}
