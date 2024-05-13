package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import org.ejml.data.DMatrixRMaj;

import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

public class KinematicsToolboxCenterOfMassCommand implements Command<KinematicsToolboxCenterOfMassCommand, KinematicsToolboxCenterOfMassMessage>
{
   private long sequenceId;
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private boolean hasDesiredVelocity;
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
   private final DMatrixRMaj weightVector = new DMatrixRMaj(3, 1);

   @Override
   public void clear()
   {
      sequenceId = 0;
      desiredPosition.setToNaN(ReferenceFrame.getWorldFrame());
      hasDesiredVelocity = false;
      desiredVelocity.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      weightVector.zero();
   }

   @Override
   public void set(KinematicsToolboxCenterOfMassCommand other)
   {
      sequenceId = other.sequenceId;
      desiredPosition.setIncludingFrame(other.desiredPosition);
      hasDesiredVelocity = other.hasDesiredVelocity;
      desiredVelocity.setIncludingFrame(other.desiredVelocity);
      selectionMatrix.set(other.selectionMatrix);
      weightVector.set(other.weightVector);
   }

   @Override
   public void setFromMessage(KinematicsToolboxCenterOfMassMessage message)
   {
      if (message == null)
      {
         clear();
         return;
      }
      sequenceId = message.getSequenceId();
      desiredPosition.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getDesiredPositionInWorld());
      hasDesiredVelocity = message.getHasDesiredLinearVelocity();
      desiredVelocity.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getDesiredLinearVelocityInWorld());
      selectionMatrix.clearSelection();
      selectionMatrix.clearSelection();
      selectionMatrix.selectXAxis(message.getSelectionMatrix().getXSelected());
      selectionMatrix.selectYAxis(message.getSelectionMatrix().getYSelected());
      selectionMatrix.selectZAxis(message.getSelectionMatrix().getZSelected());
      weightVector.reshape(3, 1);
      if (message.getWeights() == null)
      {
         weightVector.zero();
      }
      else
      {
         weightVector.set(0, 0, message.getWeights().getXWeight());
         weightVector.set(1, 0, message.getWeights().getYWeight());
         weightVector.set(2, 0, message.getWeights().getZWeight());
      }
   }

   public void setHasDesiredVelocity(boolean hasDesiredVelocity)
   {
      this.hasDesiredVelocity = hasDesiredVelocity;
   }

   public DMatrixRMaj getWeightVector()
   {
      return weightVector;
   }

   public SelectionMatrix3D getSelectionMatrix()
   {
      return selectionMatrix;
   }

   public FramePoint3D getDesiredPosition()
   {
      return desiredPosition;
   }

   public boolean getHasDesiredVelocity()
   {
      return hasDesiredVelocity;
   }

   public FrameVector3D getDesiredVelocity()
   {
      return desiredVelocity;
   }

   @Override
   public Class<KinematicsToolboxCenterOfMassMessage> getMessageClass()
   {
      return KinematicsToolboxCenterOfMassMessage.class;
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
}
