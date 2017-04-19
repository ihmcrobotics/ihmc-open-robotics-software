package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.KinematicsToolboxConfigurationMessage;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

public class KinematicsToolboxInputHelper
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final double DEFAULT_LOW_WEIGHT = 0.02;

   /**
    * Convenience method to create a {@link KinematicsToolboxRigidBodyMessage} that can be used to
    * hold the current pose of the given rigid-body.
    * <p>
    * By default the weight of the task is set to {@value #DEFAULT_LOW_WEIGHT} such that the
    * rigid-body will be held in place only if the other tasks are reachable from the current pose.
    * </p>
    * 
    * @param rigidBody the rigid-body to hold the current pose of.
    * @return the message ready to send to the {@code KinematicsToolbosModule}.
    */
   public static KinematicsToolboxRigidBodyMessage holdRigidBodyCurrentPose(RigidBody rigidBody)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(rigidBody);
      FramePose currentPose = new FramePose(rigidBody.getBodyFixedFrame());
      currentPose.changeFrame(worldFrame);

      message.setDesiredPose(currentPose);
      message.setSelectionMatrixToIdentity();
      message.setWeight(DEFAULT_LOW_WEIGHT);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);

      return message;
   }

   /**
    * Convenience method to create a {@link KinematicsToolboxRigidBodyMessage} that can be used to
    * hold the current orientation of the given rigid-body.
    * <p>
    * By default the weight of the task is set to {@value #DEFAULT_LOW_WEIGHT} such that the
    * rigid-body will be held in place only if the other tasks are reachable from the current
    * orientation.
    * </p>
    * 
    * @param rigidBody the rigid-body to hold the current orientation of.
    * @return the message ready to send to the {@code KinematicsToolbosModule}.
    */
   public static KinematicsToolboxRigidBodyMessage holdRigidBodyCurrentOrientation(RigidBody rigidBody)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(rigidBody);
      FrameOrientation currentOrientation = new FrameOrientation(rigidBody.getBodyFixedFrame());
      currentOrientation.changeFrame(worldFrame);

      message.setDesiredOrientation(currentOrientation);
      message.setSelectionMatrixForAngularControl();
      message.setWeight(DEFAULT_LOW_WEIGHT);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);

      return message;
   }

   /**
    * Create a new configuration message holding a new privileged robot configuration for the
    * {@code KinematicsToolboxController} to use by extracting the joint angles out of the given
    * {@code fullRobotModel}.
    * <p>
    * Note that the solver will update the center of mass position and foot poses in order to be
    * able to hold their configurations. Depending on the context, it may make sense to disable this
    * feature by setting to {@code false} the fields
    * {@link KinematicsToolboxConfigurationMessage#holdCurrentCenterOfMassXYPosition} and
    * {@link KinematicsToolboxConfigurationMessage#holdSupporFootPositions}.
    * </p>
    * <p>
    * Note that by sending a privileged configuration the solver will get reinitialized to start off
    * that configuration and thus may delay the convergence to the solution. It is therefore
    * preferable to send the privileged configuration as soon as possible.
    * </p>
    * 
    * @param fullRobotModel the robot that is currently at the desired privileged configuration. Not
    *           modified.
    * @param useDesiredJointAngles whether the privileged joint angles are using
    *           {@link OneDoFJoint#getqDesired()} or {@link OneDoFJoint#getQ()}.
    * @return the message containing the new privileged configuration ready to be sent to the
    *         {@code KinematicsToolboxModule}.
    */
   public static KinematicsToolboxConfigurationMessage privilegedConfigurationFromFullRobotModel(FullRobotModel fullRobotModel, boolean useDesiredJointAngles)
   {
      KinematicsToolboxConfigurationMessage message = new KinematicsToolboxConfigurationMessage();

      OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

      long[] jointNameBasedHashCodes = new long[oneDoFJoints.length];
      float[] privilegedJointAngles = new float[oneDoFJoints.length];

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         jointNameBasedHashCodes[i] = oneDoFJoints[i].getNameBasedHashCode();

         if (useDesiredJointAngles)
            privilegedJointAngles[i] = (float) oneDoFJoints[i].getqDesired();
         else
            privilegedJointAngles[i] = (float) oneDoFJoints[i].getQ();
      }

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      Point3D privilegedRootJointPosition = new Point3D();
      rootJoint.getTranslation(privilegedRootJointPosition);
      Quaternion privilegedRootJointOrientation = new Quaternion();
      rootJoint.getRotation(privilegedRootJointOrientation);

      message.setPrivilegedRobotConfiguration(privilegedRootJointPosition, privilegedRootJointOrientation, jointNameBasedHashCodes, privilegedJointAngles);
      message.setDestination(PacketDestination.KINEMATICS_TOOLBOX_MODULE);

      return message;
   }
}
