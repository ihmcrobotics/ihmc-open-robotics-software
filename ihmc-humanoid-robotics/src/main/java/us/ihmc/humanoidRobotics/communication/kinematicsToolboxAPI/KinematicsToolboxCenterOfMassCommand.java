package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;

public class KinematicsToolboxCenterOfMassCommand implements Command<KinematicsToolboxCenterOfMassCommand, KinematicsToolboxCenterOfMassMessage>
{
   private long sequenceId;
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private boolean hasDesiredVelocity;
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
   private final WeightMatrix3D weightMatrix = new WeightMatrix3D();

   private double linearRateLimitation;

   @Override
   public void clear()
   {
      sequenceId = 0;
      desiredPosition.setToNaN(ReferenceFrame.getWorldFrame());
      hasDesiredVelocity = false;
      desiredVelocity.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      weightMatrix.clear();
      linearRateLimitation = -1.0;
   }

   @Override
   public void set(KinematicsToolboxCenterOfMassCommand other)
   {
      sequenceId = other.sequenceId;
      desiredPosition.setIncludingFrame(other.desiredPosition);
      hasDesiredVelocity = other.hasDesiredVelocity;
      desiredVelocity.setIncludingFrame(other.desiredVelocity);
      selectionMatrix.set(other.selectionMatrix);
      weightMatrix.set(other.weightMatrix);
      linearRateLimitation = other.linearRateLimitation;
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
      weightMatrix.clear();
      weightMatrix.setWeights(message.getWeights().getXWeight(), message.getWeights().getYWeight(), message.getWeights().getZWeight());
      linearRateLimitation = message.getLinearRateLimitation();
   }

   public void setHasDesiredVelocity(boolean hasDesiredVelocity)
   {
      this.hasDesiredVelocity = hasDesiredVelocity;
   }

   public WeightMatrix3D getWeightMatrix()
   {
      return weightMatrix;
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

   public double getLinearRateLimitation()
   {
      return linearRateLimitation;
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
