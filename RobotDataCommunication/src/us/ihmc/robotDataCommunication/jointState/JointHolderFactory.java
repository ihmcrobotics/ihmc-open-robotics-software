package us.ihmc.robotDataCommunication.jointState;

import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

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
