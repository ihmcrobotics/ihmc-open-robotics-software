package us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxCenterOfMassMessage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

public class KinematicsToolboxCenterOfMassCommand implements Command<KinematicsToolboxCenterOfMassCommand, KinematicsToolboxCenterOfMassMessage>
{
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();
   private final DenseMatrix64F weightVector = new DenseMatrix64F(3, 1);

   @Override
   public void clear()
   {
      desiredPosition.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.resetSelection();
      weightVector.zero();
   }

   @Override
   public void set(KinematicsToolboxCenterOfMassCommand other)
   {
      desiredPosition.setIncludingFrame(other.desiredPosition);
      selectionMatrix.set(other.selectionMatrix);
      weightVector.set(other.weightVector);
   }

   @Override
   public void set(KinematicsToolboxCenterOfMassMessage message)
   {
      if (message == null)
      {
         clear();
         return;
      }
      desiredPosition.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.desiredPositionInWorld);
      selectionMatrix.clearSelection();
      selectionMatrix.clearSelection();
      selectionMatrix.selectXAxis(message.selectionMatrix.xSelected);
      selectionMatrix.selectYAxis(message.selectionMatrix.ySelected);
      selectionMatrix.selectZAxis(message.selectionMatrix.zSelected);
      weightVector.reshape(3, 1);
      if (message.weights == null)
      {
         weightVector.zero();
      }
      else
      {
         weightVector.set(0, 0, message.weights.getXWeight());
         weightVector.set(1, 0, message.weights.getYWeight());
         weightVector.set(2, 0, message.weights.getZWeight());
      }
   }

   public DenseMatrix64F getWeightVector()
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
}
