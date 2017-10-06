package us.ihmc.robotDataLogger.jointState;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

public class JointHolderFactory
{
   public static JointHolder getJointHolder(InverseDynamicsJoint joint)
   {
      if(joint instanceof SixDoFJoint)
      {
         return new SiXDoFJointHolder((SixDoFJoint) joint);
      }
      else if(joint instanceof OneDoFJoint)
      {
         return new OneDoFJointHolder((OneDoFJoint) joint);
      }
      else
      {
         throw new RuntimeException("Joint type " + joint.getClass().getSimpleName() + " not supported for communication");
      }
   }
}
