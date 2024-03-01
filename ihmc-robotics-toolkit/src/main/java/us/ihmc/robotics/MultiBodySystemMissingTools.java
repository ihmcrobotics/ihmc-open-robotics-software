package us.ihmc.robotics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;

import javax.annotation.Nullable;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MultiBodySystemMissingTools
{
   public static void copyOneDoFJointsConfiguration(JointBasics[] source, JointBasics[] destination)
   {
      OneDoFJointBasics[] oneDofJoints1 = MultiBodySystemTools.filterJoints(source, OneDoFJointBasics.class);
      OneDoFJointBasics[] oneDofJoints2 = MultiBodySystemTools.filterJoints(destination, OneDoFJointBasics.class);

      if (oneDofJoints1.length != oneDofJoints2.length)
      {
         throw new IllegalArgumentException("The lists of joints must be the same length. %d != %d".formatted(oneDofJoints1.length,
                                                                                                              oneDofJoints2.length));
      }

      for (int i = 0; i < oneDofJoints1.length; i++)
      {
         oneDofJoints2[i].setJointConfiguration(oneDofJoints1[i]);
      }
   }

   /**
    * You want an elevator if you want to move the base around in world.
    * Otherwise, it's stuck there.
    *
    * See {@link #getDetachedCopyOfSubtree}
    */
   public static RigidBodyBasics getDetachedCopyOfSubtreeWithElevator(RigidBodyBasics rootBodyToDetach)
   {
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint floatingJoint = new SixDoFJoint(rootBodyToDetach.getName(), elevator);
      RigidBodyBasics clonedRootBody = MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER.cloneRigidBody(rootBodyToDetach,
                                                                                                       null,
                                                                                                       "",
                                                                                                       floatingJoint);
      cloneSubtree(rootBodyToDetach, clonedRootBody, "", null);
      return elevator;
   }

   /**
    * You want an elevator if you want to move the base around in world.
    * Otherwise, it's stuck there.
    *
    * See {@link #getDetachedCopyOfSubtree}
    */
   public static RigidBodyBasics getDetachedCopyOfSubtreeWithElevator(RigidBodyBasics rootBodyToDetach,
                                                                      OneDoFJointBasics childJointToFollow,
                                                                      @Nullable String endRigidBodyName)
   {
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint floatingJoint = new SixDoFJoint(rootBodyToDetach.getName(), elevator);
      RigidBodyBasics clonedChest = MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER.cloneRigidBody(rootBodyToDetach,
                                                                                                       null,
                                                                                                       "",
                                                                                                       floatingJoint);
      JointBasics clonedFirstShoulderJoint = MultiBodySystemFactories.DEFAULT_JOINT_BUILDER.cloneJoint(childJointToFollow, "", clonedChest);
      RigidBodyBasics clonedFirstShoulderLink = MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER.cloneRigidBody(childJointToFollow.getSuccessor(),
                                                                                                                   null,
                                                                                                                   "",
                                                                                                                   clonedFirstShoulderJoint);
      cloneSubtree(childJointToFollow.getSuccessor(), clonedFirstShoulderLink, "", endRigidBodyName);
      return elevator;
   }

   /**
    * This is useful to get a detached copy of an arm or a leg, or perhaps a finger,
    * for running IK over it.
    *
    * @param rootBodyToDetach not the elevator, but like the chest, or the pelvis
    * @param cloneStationaryFrame Usually world frame, can be null
    * @param childJointToFollow for the chest, like one of the shoulders or something
    * @param endRigidBodyName null or the name of the rigid body to stop at, which will be included with no children joints
    */
   public static RigidBodyBasics getDetachedCopyOfSubtree(RigidBodyBasics rootBodyToDetach,
                                                          @Nullable ReferenceFrame cloneStationaryFrame,
                                                          OneDoFJointBasics childJointToFollow,
                                                          @Nullable String endRigidBodyName)
   {
      RigidBodyBasics clonedChest = MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER.cloneRigidBody(rootBodyToDetach, cloneStationaryFrame, "", null);
      JointBasics clonedFirstShoulderJoint = MultiBodySystemFactories.DEFAULT_JOINT_BUILDER.cloneJoint(childJointToFollow, "", clonedChest);
      RigidBodyBasics clonedFirstShoulderLink = MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER.cloneRigidBody(childJointToFollow.getSuccessor(),
                                                                                                                   null,
                                                                                                                   "",
                                                                                                                   clonedFirstShoulderJoint);
      cloneSubtree(childJointToFollow.getSuccessor(), clonedFirstShoulderLink, "", endRigidBodyName);
      return clonedChest;
   }

   /**
    * Sad to have to copy all this over, but it was private :(
    * https://github.com/ihmcrobotics/mecano/issues/12
    */
   private static void cloneSubtree(RigidBodyReadOnly originalStart, RigidBodyBasics cloneStart, String cloneSuffix, String endRigidBodyName)
   {
      MultiBodySystemFactories.RigidBodyBuilder rigidBodyBuilder = MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER;
      MultiBodySystemFactories.JointBuilder jointBuilder = MultiBodySystemFactories.DEFAULT_JOINT_BUILDER;

      // Changed this to String so that trees with bodies extending RigidBodyBasics will still work
      Map<String, RigidBodyBasics> originalToCloneBodyMap = new HashMap<>();
      originalToCloneBodyMap.put(originalStart.getName(), cloneStart);

      List<JointBasics> loopClosureCloneJoints = new ArrayList<>();
      List<JointReadOnly> loopClosureOriginalJoints = new ArrayList<>();

      for (JointReadOnly originalJoint : originalStart.childrenSubtreeIterable())
      {
         RigidBodyReadOnly originalPredecessor = originalJoint.getPredecessor();

         if (endRigidBodyName == null || !originalPredecessor.getName().equals(endRigidBodyName))
         {
            // Retrieve the right predecessor for the joint to clone. The map has to contain the clone predecessor.
            RigidBodyBasics clonePredecessor = originalToCloneBodyMap.get(originalPredecessor.getName());

            // Clone the joint
            JointBasics cloneJoint = jointBuilder.cloneJoint(originalJoint, cloneSuffix, clonePredecessor);

            if (originalJoint.isLoopClosure())
            { // We rely on the iterator to stop at the loop closure joint.
               loopClosureCloneJoints.add(cloneJoint);
               loopClosureOriginalJoints.add(originalJoint);
               // We will complete their setup at the end to ensure the successors are already created.
               continue;
            }

            // Clone the successor
            RigidBodyReadOnly originalSuccessor = originalJoint.getSuccessor();
            RigidBodyBasics cloneSuccessor = rigidBodyBuilder.cloneRigidBody(originalSuccessor, null, cloneSuffix, cloneJoint);
            originalToCloneBodyMap.put(originalSuccessor.getName(), cloneSuccessor);
         }
         else
         {
            break;
         }
      }

      for (int loopClosureIndex = 0; loopClosureIndex < loopClosureCloneJoints.size(); loopClosureIndex++)
      {
         JointBasics cloneJoint = loopClosureCloneJoints.get(loopClosureIndex);
         JointReadOnly originalJoint = loopClosureOriginalJoints.get(loopClosureIndex);

         RigidBodyBasics cloneSuccessor = originalToCloneBodyMap.get(originalJoint.getSuccessor().getName());
         RigidBodyTransform cloneTransform = new RigidBodyTransform(originalJoint.getLoopClosureFrame().getTransformToParent());
         cloneTransform.invert();
         cloneJoint.setupLoopClosure(cloneSuccessor, cloneTransform);
      }
   }


   public static <J extends JointReadOnly> J[] getSubtreeJointArray(Class<J> jointTypeFilter, RigidBodyReadOnly start)
   {
      ArrayList<J> joints = new ArrayList<>();
      SubtreeStreams.fromChildren(jointTypeFilter, start).forEach(joints::add);
      return joints.toArray((J[]) Array.newInstance(jointTypeFilter, 0));
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

   public static MultiBodySystemBasics createSingleBodySystem(RigidBodyBasics singleBody)
   {
      List<? extends JointBasics> allJoints = new ArrayList<>();
      List<? extends JointBasics> jointsToConsider = new ArrayList<>();
      List<? extends JointBasics> jointsToIgnore = new ArrayList<>();
      JointMatrixIndexProvider jointMatrixIndexProvider = JointMatrixIndexProvider.toIndexProvider(jointsToConsider);

      return new MultiBodySystemBasics()
      {
         @Override
         public RigidBodyBasics getRootBody()
         {
            return singleBody;
         }

         @Override
         public List<? extends JointBasics> getAllJoints()
         {
            return allJoints;
         }

         @Override
         public List<? extends JointBasics> getJointsToConsider()
         {
            return jointsToConsider;
         }

         @Override
         public List<? extends JointBasics> getJointsToIgnore()
         {
            return jointsToIgnore;
         }

         @Override
         public JointMatrixIndexProvider getJointMatrixIndexProvider()
         {
            return jointMatrixIndexProvider;
         }
      };
   }
}
