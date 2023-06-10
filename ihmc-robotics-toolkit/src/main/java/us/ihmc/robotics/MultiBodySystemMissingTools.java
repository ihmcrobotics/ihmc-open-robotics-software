package us.ihmc.robotics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import java.lang.reflect.Array;
import java.util.ArrayList;

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

   /**
    * This is useful to get a detached copy of an arm or a leg, or perhaps a finger,
    * for running IK over it.
    *
    * @param rootBodyToDetach not the elevator, but like the chest, or the pelvis
    * @param childJointToFollow for the chest, like one of the shoulders or something
    */
   public static RigidBody getDetachedCopyOfSubtree(RigidBodyBasics rootBodyToDetach, OneDoFJointBasics childJointToFollow)
   {
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint floatingJoint = new SixDoFJoint(rootBodyToDetach.getName(), elevator);
      RigidBodyBasics clonedChest = MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER.cloneRigidBody(rootBodyToDetach, null, "", floatingJoint);
      JointBasics clonedFirstShoulderJoint = MultiBodySystemFactories.DEFAULT_JOINT_BUILDER.cloneJoint(childJointToFollow, "", clonedChest);
      RigidBodyBasics clonedFirstShoulderLink = MultiBodySystemFactories.cloneSubtree(childJointToFollow.getSuccessor(), "");
      clonedFirstShoulderJoint.setSuccessor(clonedFirstShoulderLink);
      return elevator;
   }

   public static <J extends JointReadOnly> J[] getSubtreeJointArray(Class<J> jointTypeFilter, RigidBodyReadOnly start)
   {
      ArrayList<J> joints = new ArrayList<>();
      SubtreeStreams.fromChildren(jointTypeFilter, start).forEach(joints::add);
      return joints.toArray((J[]) Array.newInstance(jointTypeFilter, 0));
   }
}
