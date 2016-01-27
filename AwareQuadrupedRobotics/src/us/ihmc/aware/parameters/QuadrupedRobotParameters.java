package us.ihmc.aware.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFParameters;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.aware.vmc.QuadrupedContactForceLimits;
import us.ihmc.aware.vmc.QuadrupedJointLimits;
import us.ihmc.quadrupedRobotics.parameters.*;

public abstract class QuadrupedRobotParameters
{
   private final QuadrupedJointLimits jointLimits = new QuadrupedJointLimits();
   private final QuadrupedContactForceLimits contactForceLimits = new QuadrupedContactForceLimits();
   private final QuadrupedStandPrepParameters standPrepParameters = new DefaultQuadrupedStandPrepParameters();
   private final QuadrupedVirtualModelBasedStandParameters virtualModelBasedStandParameters = new DefaultQuadrupedVirtualModelBasedStandParameters();
   private final QuadrupedVirtualModelBasedStepParameters virtualModelBasedStepParameters = new DefaultQuadrupedVirtualModelBasedStepParameters();

   public abstract SDFRobot createSdfRobot();

   public abstract QuadrupedJointNameMap getJointMap();
   
   public abstract SDFFullRobotModel createFullRobotModel();

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

   public QuadrupedVirtualModelBasedStepParameters getQuadrupedVirtualModelBasedStepParameters()
   {
      return virtualModelBasedStepParameters;
   }
}
