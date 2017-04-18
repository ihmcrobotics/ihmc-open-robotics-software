package us.ihmc.communication.kinematicsToolboxAPI;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.KinematicsToolboxCenterOfMassMessage;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class KinematicsToolboxCenterOfMassCommand implements Command<KinematicsToolboxCenterOfMassCommand, KinematicsToolboxCenterOfMassMessage>
{
   private final FramePoint desiredPosition = new FramePoint();
   private final DenseMatrix64F selectionMatrix = CommonOps.identity(3);
   private final DenseMatrix64F weightVector = new DenseMatrix64F(3, 1);

   @Override
   public void clear()
   {
      desiredPosition.setToNaN(ReferenceFrame.getWorldFrame());
      selectionMatrix.reshape(3, 6);
      CommonOps.setIdentity(selectionMatrix);
      for (int i = 0; i < 3; i++)
         selectionMatrix.set(i, i, 1.0);
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
      message.getDesiredPosition(desiredPosition);
      message.getSelectionMatrix(selectionMatrix);
      message.getWeightVector(weightVector);
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
