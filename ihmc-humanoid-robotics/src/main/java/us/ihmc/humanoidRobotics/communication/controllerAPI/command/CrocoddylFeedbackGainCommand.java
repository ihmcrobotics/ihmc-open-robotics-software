package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.CrocoddylFeedbackGainMessage;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.communication.controllerAPI.command.Command;

public class CrocoddylFeedbackGainCommand implements Command<CrocoddylFeedbackGainCommand, CrocoddylFeedbackGainMessage>
{
   private int sequenceId = -1;
   private final DMatrixRMaj gainMatrix = new DMatrixRMaj(0, 0);

   @Override
   public void clear()
   {
      this.sequenceId = -1;
      gainMatrix.reshape(0, 0);
      gainMatrix.zero();
   }

   @Override
   public void setFromMessage(CrocoddylFeedbackGainMessage message)
   {
      this.sequenceId = (int) message.getUniqueId();

      gainMatrix.reshape((int) message.getNu(), (int) message.getNx());
      for (int element = 0; element < gainMatrix.getNumElements(); element++)
      {
         gainMatrix.set(element, message.getData().get(element));
      }
   }

   @Override
   public Class<CrocoddylFeedbackGainMessage> getMessageClass()
   {
      return CrocoddylFeedbackGainMessage.class;
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

   @Override
   public void set(CrocoddylFeedbackGainCommand other)
   {
      this.sequenceId = other.sequenceId;
      this.gainMatrix.set(other.gainMatrix);
   }

   public DMatrixRMaj getGainMatrix()
   {
      return gainMatrix;
   }
}
