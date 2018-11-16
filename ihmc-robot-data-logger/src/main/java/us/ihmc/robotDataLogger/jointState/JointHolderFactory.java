package us.ihmc.robotDataLogger.jointState;

import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class JointHolderFactory
{
   public static JointHolder getJointHolder(JointBasics joint)
   {
      if(joint instanceof SixDoFJoint)
      {
         return new SiXDoFJointHolder((SixDoFJoint) joint);
      }
      else if(joint instanceof OneDoFJointBasics)
      {
         return new OneDoFJointHolder((OneDoFJointBasics) joint);
      }
      else
      {
         throw new RuntimeException("Joint type " + joint.getClass().getSimpleName() + " not supported for communication");
      }
   }
}
