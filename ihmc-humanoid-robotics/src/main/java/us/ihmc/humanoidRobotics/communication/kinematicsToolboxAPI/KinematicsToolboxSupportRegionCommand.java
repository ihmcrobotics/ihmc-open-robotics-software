package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import java.util.List;

import toolbox_msgs.msg.dds.KinematicsToolboxSupportRegionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsToolboxSupportRegionCommand implements Command<KinematicsToolboxSupportRegionCommand, KinematicsToolboxSupportRegionMessage>
{
   private long sequenceId;
   private double centerOfMassMargin = -1.0;
   private final RecyclingArrayList<KinematicsToolboxSupportRegionVertex> contactPoints = new RecyclingArrayList<>(KinematicsToolboxSupportRegionVertex::new);

   @Override
   public void clear()
   {
      sequenceId = 0;
      centerOfMassMargin = -1.0;
      contactPoints.clear();
   }

   @Override
   public void set(KinematicsToolboxSupportRegionCommand other)
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
   public void setFromMessage(KinematicsToolboxSupportRegionMessage message)
   {
      set(message, null);
   }

   public void set(KinematicsToolboxSupportRegionMessage message, ReferenceFrameHashCodeResolver referenceFrameResolver)
   {
      sequenceId = message.getSequenceId();
      centerOfMassMargin = message.getCenterOfMassMargin();
      contactPoints.clear();
      boolean hasCustomReferenceFrames = message.getSupportRegionVertexFrames().size() == message.getSupportRegionVertices().size();

      for (int i = 0; i < message.getSupportRegionVertices().size(); i++)
      {
         ReferenceFrame referenceFrame = hasCustomReferenceFrames ? referenceFrameResolver.getReferenceFrame(message.getSupportRegionVertexFrames().get(i)) : ReferenceFrame.getWorldFrame();
         Point3D supportRegionVertex = message.getSupportRegionVertices().get(i);
         contactPoints.add().set(referenceFrame, supportRegionVertex);
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

   public KinematicsToolboxSupportRegionVertex getContactPoint(int index)
   {
      return contactPoints.get(index);
   }

   public List<KinematicsToolboxSupportRegionVertex> getContactPoints()
   {
      return contactPoints;
   }

   @Override
   public Class<KinematicsToolboxSupportRegionMessage> getMessageClass()
   {
      return KinematicsToolboxSupportRegionMessage.class;
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


   public static class KinematicsToolboxSupportRegionVertex
   {
      private ReferenceFrame referenceFrame;
      private final FramePoint3D vertexPosition = new FramePoint3D();

      private void set(ReferenceFrame referenceFrame, Point3DReadOnly positionInBodyFrame)
      {
         this.referenceFrame = referenceFrame;
         vertexPosition.setIncludingFrame(referenceFrame, positionInBodyFrame);
      }

      private void set(KinematicsToolboxSupportRegionVertex other)
      {
         referenceFrame = other.referenceFrame;
         vertexPosition.setIncludingFrame(other.vertexPosition);
      }

      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      public FramePoint3DReadOnly getVertexPosition()
      {
         return vertexPosition;
      }
   }
}
