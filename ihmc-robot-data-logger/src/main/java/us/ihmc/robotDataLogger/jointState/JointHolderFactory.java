package us.ihmc.robotDataLogger.jointState;

import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;

public class JointHolderFactory
{
   public static JointHolder getJointHolder(JointBasics joint)
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
