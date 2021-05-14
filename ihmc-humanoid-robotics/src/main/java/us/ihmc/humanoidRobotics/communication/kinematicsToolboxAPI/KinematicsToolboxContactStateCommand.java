package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.List;

import controller_msgs.msg.dds.KinematicsToolboxContactStateMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;

public class KinematicsToolboxContactStateCommand implements Command<KinematicsToolboxContactStateCommand, KinematicsToolboxContactStateMessage>
{
   private long sequenceId;
   private double centerOfMassMargin = -1.0;
   private final RecyclingArrayList<KinematicsToolboxContactPoint> contactPoints = new RecyclingArrayList<>(KinematicsToolboxContactPoint::new);

   @Override
   public void clear()
   {
      sequenceId = 0;
      centerOfMassMargin = -1.0;
      contactPoints.clear();
   }

   @Override
   public void set(KinematicsToolboxContactStateCommand other)
   {
      sequenceId = other.sequenceId;
      centerOfMassMargin = other.centerOfMassMargin;
      contactPoints.clear();

      for (int i = 0; i < other.getNumberOfContacts(); i++)
      {
         contactPoints.add().set(other.getContactPoint(i));
      }
   }

   @Override
   public void setFromMessage(KinematicsToolboxContactStateMessage message)
   {
      set(message, null);
   }

   public void set(KinematicsToolboxContactStateMessage message, RigidBodyHashCodeResolver rigidBodyHashCodeResolver)
   {
      sequenceId = message.getSequenceId();
      centerOfMassMargin = message.getCenterOfMassMargin();
      contactPoints.clear();

      for (int i = 0; i < message.getContactPointsInBodyFrame().size(); i++)
      {
         RigidBodyBasics rigidBody = rigidBodyHashCodeResolver.castAndGetRigidBody(message.getContactingBodyIds().get(i));
         Point3D contactPointPosition = message.getContactPointsInBodyFrame().get(i);
         contactPoints.add().set(rigidBody, contactPointPosition);
      }
   }

   public double getCenterOfMassMargin()
   {
      return centerOfMassMargin;
   }

   public int getNumberOfContacts()
   {
      return contactPoints.size();
   }

   public KinematicsToolboxContactPoint getContactPoint(int index)
   {
      return contactPoints.get(index);
   }

   public List<KinematicsToolboxContactPoint> getContactPoints()
   {
      return contactPoints;
   }

   @Override
   public Class<KinematicsToolboxContactStateMessage> getMessageClass()
   {
      return KinematicsToolboxContactStateMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   public static class KinematicsToolboxContactPoint
   {
      private RigidBodyBasics rigidBody;
      private final FramePoint3D position = new FramePoint3D();

      private void set(RigidBodyBasics rigidBody, Point3DReadOnly positionInBodyFrame)
      {
         this.rigidBody = rigidBody;
         position.setIncludingFrame(rigidBody.getBodyFixedFrame(), positionInBodyFrame);
      }

      private void set(KinematicsToolboxContactPoint other)
      {
         rigidBody = other.rigidBody;
         position.setIncludingFrame(other.position);
      }

      public RigidBodyBasics getRigidBody()
      {
         return rigidBody;
      }

      public FramePoint3DReadOnly getPosition()
      {
         return position;
      }
   }
}
