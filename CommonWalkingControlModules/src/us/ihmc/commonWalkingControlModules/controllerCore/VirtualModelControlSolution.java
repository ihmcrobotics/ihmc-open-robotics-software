package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.Map;

public class VirtualModelControlSolution
{
   private final OneDoFJoint[] jointsToCompute;
   private final Map<OneDoFJoint, Double> jointTorques;

   public VirtualModelControlSolution(OneDoFJoint[] jointsToCompute, Map<OneDoFJoint, Double> jointTorques)
   {
      this.jointsToCompute = jointsToCompute;
      this.jointTorques = jointTorques;
   }

   public OneDoFJoint[] getJointsToCompute()
   {
      return jointsToCompute;
   }

   public Map<OneDoFJoint, Double> getJointTorques()
   {
      return jointTorques;
   }
}
