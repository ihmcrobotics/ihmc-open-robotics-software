package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.LoadBearingMessage;
import us.ihmc.communication.controllerAPI.command.Command;
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

   /** Sets the transform of the contact frame in the frame of the end effector body. */
   public final Point3D contactPointInBodyFrame = new Point3D();

   /** Sets the contact normal used by the controller to load the contact point. */
   private final Vector3D contactNormalInWorldFrame = new Vector3D();

   public boolean getLoad()
   {
      return load;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public Point3D getContactPointInBodyFrame()
   {
      return contactPointInBodyFrame;
   }

   public Vector3D getContactNormalInWorldFrame()
   {
      return contactNormalInWorldFrame;
   }

   @Override
   public void set(LoadBearingCommand other)
   {
      sequenceId = other.sequenceId;
      load = other.getLoad();
      coefficientOfFriction = other.getCoefficientOfFriction();
      contactPointInBodyFrame.set(other.getContactPointInBodyFrame());
      contactNormalInWorldFrame.set(other.getContactNormalInWorldFrame());
   }

   @Override
   public void setFromMessage(LoadBearingMessage message)
   {
      sequenceId = message.getSequenceId();
      load = message.getLoad();
      coefficientOfFriction = message.getCoefficientOfFriction();
      contactPointInBodyFrame.set(message.getContactPointInBodyFrame());
      contactNormalInWorldFrame.set(message.getContactNormalInWorldFrame());
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      load = false;
      coefficientOfFriction = 0.0;
      contactPointInBodyFrame.setToZero();
      contactNormalInWorldFrame.setToZero();
   }

   @Override
   public boolean isCommandValid()
   {
      if (coefficientOfFriction <= 0.0)
         return false;
      if (contactPointInBodyFrame.containsNaN())
         return false;
      if (contactNormalInWorldFrame.containsNaN())
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
