package us.ihmc.robotDataVisualizer.visualizer;

import us.ihmc.robotDataLogger.jointState.OneDoFState;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class OneDegreeOfFreedomJointUpdater extends JointUpdater
{

   private final OneDegreeOfFreedomJoint joint;
   private final OneDoFState jointState;

   public OneDegreeOfFreedomJointUpdater(OneDegreeOfFreedomJoint joint, OneDoFState jointState)
   {
      this.joint = joint;
      this.jointState = jointState;
   }

   @Override
   public void update()
   {

      double q = jointState.getQ();
      double qd = jointState.getQd();
      if (!Double.isNaN(q))
      {
         joint.setQ(q);
      }
      if (!Double.isNaN(qd))
      {
         joint.setQd(qd);
      }
   }
}
