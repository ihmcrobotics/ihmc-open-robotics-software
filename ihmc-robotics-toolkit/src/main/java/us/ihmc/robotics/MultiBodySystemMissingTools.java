package us.ihmc.robotics;

<<<<<<<HEAD import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

=======
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.util.ArrayList;
import java.util.List;

public class MultiBodySystemMissingTools
{
   public static void copyOneDoFJointsConfiguration(JointBasics[] source, JointBasics[] destination)
   {
      OneDoFJointBasics[] oneDofJoints1 = MultiBodySystemTools.filterJoints(source, OneDoFJointBasics.class);
      OneDoFJointBasics[] oneDofJoints2 = MultiBodySystemTools.filterJoints(destination, OneDoFJointBasics.class);

      for (int i = 0; i < oneDofJoints1.length; i++)
      {
         oneDofJoints2[i].setJointConfiguration(oneDofJoints1[i]);
      }
   }

   public static List<JointBasics> getSubtreeJointsIncludingFourBars(RigidBodyBasics rootBody)
   {
      List<JointBasics> joints = new ArrayList<>();

      for (JointBasics joint : rootBody.childrenSubtreeIterable())
      {
         if (joint instanceof CrossFourBarJoint)
         {
            joints.addAll(((CrossFourBarJoint) joint).getFourBarFunction().getLoopJoints());
         }
         else
         {
            joints.add(joint);
         }
      }

      return joints;
   }
}
