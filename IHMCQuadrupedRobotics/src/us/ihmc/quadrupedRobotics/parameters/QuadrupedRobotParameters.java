package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFParameters;
import us.ihmc.SdfLoader.SDFRobot;

public abstract class QuadrupedRobotParameters
{
   private final QuadrupedStandPrepParameters standPrepParams = new DefaultQuadrupedStandPrepParameters();
   private final QuadrupedVirtualModelParameters vmcParams = new DefaultQuadrupedVirtualModelParameters();
   private final QuadrupedVirtualModelBasedStandParameters vmcStandParams = new DefaultQuadrupedVirtualModelBasedStandParameters();

   public abstract SDFRobot createSdfRobot();

   public abstract QuadrupedJointNameMap getJointMap();
   
   public abstract QuadrupedJointLimits getJointLimits();

   public abstract SDFFullRobotModel createFullRobotModel();

   public abstract String getModelName();

   public abstract SDFParameters getSdfParameters();

   public abstract QuadrupedPhysicalProperties getPhysicalProperties();

   public abstract QuadrupedActuatorParameters getActuatorParameters();

   public abstract QuadrupedControllerParameters getQuadrupedControllerParameters();
   
   public QuadrupedStandPrepParameters getQuadrupedStandPrepParameters()
   {
      return standPrepParams;
   }
   
   public QuadrupedVirtualModelParameters getQuadrupedVirtualModelParameters()
   {
      return vmcParams;
   }

   public QuadrupedVirtualModelBasedStandParameters getQuadrupedVMCStandParameters()
   {
      return vmcStandParams;
   }
}
