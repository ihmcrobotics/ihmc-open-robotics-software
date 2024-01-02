package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.LoadBearingMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public final class LoadBearingCommand implements Command<LoadBearingCommand, LoadBearingMessage>
{
   private long sequenceId;
   /** If set to true this will load the contact point. Otherwise the rigid body will stop bearing load. */
   private boolean load = false;

   /** Sets the coefficient of friction that the controller will use for the contact point. */
   private double coefficientOfFriction = 0.0;

   /** Pose of the contact frame, expressed in the frame of the end effector body
    * The contact frame origin is the contact point and the z-axis of contact frame is the contact normal (points into the environment) */
   public final Pose3D contactPoseInBodyFrame = new Pose3D();

   public boolean getLoad()
   {
      return load;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public Pose3D getContactPoseInBodyFrame()
   {
      return contactPoseInBodyFrame;
   }

   @Override
   public void set(LoadBearingCommand other)
   {
      sequenceId = other.sequenceId;
      load = other.getLoad();
      coefficientOfFriction = other.getCoefficientOfFriction();
      contactPoseInBodyFrame.set(other.contactPoseInBodyFrame);
   }

   @Override
   public void setFromMessage(LoadBearingMessage message)
   {
      sequenceId = message.getSequenceId();
      load = message.getLoad();
      coefficientOfFriction = message.getCoefficientOfFriction();
      contactPoseInBodyFrame.set(message.getContactPoseInBodyFrame());
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      load = false;
      coefficientOfFriction = 0.0;
      contactPoseInBodyFrame.setToZero();
   }

   @Override
   public boolean isCommandValid()
   {
      if (coefficientOfFriction <= 0.0)
         return false;
      if (contactPoseInBodyFrame.containsNaN())
         return false;
      return true;
   }

   @Override
   public Class<LoadBearingMessage> getMessageClass()
   {
      return LoadBearingMessage.class;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
