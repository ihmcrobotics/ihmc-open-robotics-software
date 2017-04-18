package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.KinematicsToolboxInputMessage;
import us.ihmc.communication.packets.KinematicsToolboxRigidBodyMessage;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
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
    * @return the message ready to pack into a {@link KinematicsToolboxInputMessage}.
    */
   public static KinematicsToolboxRigidBodyMessage holdRigidBodyCurrentPose(RigidBody rigidBody)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(rigidBody);
      FramePose currentPose = new FramePose(rigidBody.getBodyFixedFrame());
      currentPose.changeFrame(worldFrame);

      message.setDesiredPose(currentPose);
      message.setSelectionMatrixToIdentity();
      message.setWeight(DEFAULT_LOW_WEIGHT);

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
    * @return the message ready to pack into a {@link KinematicsToolboxInputMessage}.
    */
   public static KinematicsToolboxRigidBodyMessage holdRigidBodyCurrentOrientation(RigidBody rigidBody)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage(rigidBody);
      FrameOrientation currentOrientation = new FrameOrientation(rigidBody.getBodyFixedFrame());
      currentOrientation.changeFrame(worldFrame);

      message.setDesiredOrientation(currentOrientation);
      message.setSelectionMatrixForAngularControl();
      message.setWeight(DEFAULT_LOW_WEIGHT);

      return message;
   }
}
