package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.CrocoddylControlMessage;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.communication.controllerAPI.command.Command;

public class CrocoddylControlCommand implements Command<CrocoddylControlCommand, CrocoddylControlMessage>
{
   private int sequenceID = -1;

   private CrocoddylControlInputType inputType = null;
   private CrocoddylControlParameterization parameterization = null;

   private final DMatrixRMaj control = new DMatrixRMaj(0, 0);
   private final CrocoddylFeedbackGainCommand feedbackGainCommand = new CrocoddylFeedbackGainCommand();

   @Override
   public void clear()
   {
      sequenceID = -1;
      inputType = null;
      parameterization = null;
      control.reshape(0, 0);
      control.zero();
      feedbackGainCommand.clear();
   }

   @Override
   public void setFromMessage(CrocoddylControlMessage message)
   {
      sequenceID = (int) message.getUniqueId();

      inputType = CrocoddylControlInputType.fromByte(message.getInput());
      parameterization = CrocoddylControlParameterization.fromByte(message.getParametrization());

      control.reshape(message.getU().size(), 1);
      for (int element = 0; element < message.getU().size(); element++)
         control.set(element, message.getU().get(element));

      feedbackGainCommand.setFromMessage(message.getGain());
   }

   @Override
   public Class<CrocoddylControlMessage> getMessageClass()
   {
      return CrocoddylControlMessage.class
            ;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceID;
   }

   @Override
   public void set(CrocoddylControlCommand other)
   {
      this.sequenceID = other.sequenceID;
      this.inputType = other.inputType;
      this.parameterization = other.parameterization;
      this.control.set(other.control);
      this.feedbackGainCommand.set(other.feedbackGainCommand);
   }

   public CrocoddylControlInputType getInputType()
   {
      return inputType;
   }

   public CrocoddylControlParameterization getParameterization()
   {
      return parameterization;
   }

   public DMatrixRMaj getControl()
   {
      return control;
   }

   public CrocoddylFeedbackGainCommand getFeedbackGainCommand()
   {
      return feedbackGainCommand;
   }

   public enum CrocoddylControlInputType
   {
      EFFORT, ACCELERATION_CONTACT_FORCE;

      public static CrocoddylControlInputType fromByte(int value)
      {
         return switch (value)
         {
            case 0 -> EFFORT;
            case 1 -> ACCELERATION_CONTACT_FORCE;
            default -> null;
         };
      }

      public int toByte()
      {
         return switch (this)
         {
            case EFFORT -> 0;
            case ACCELERATION_CONTACT_FORCE -> 1;
            default -> -1;
         };
      }
   }

   public enum CrocoddylControlParameterization
   {
      POLY_ZERO, POLY_ONE, POLY_TWO;

      public static CrocoddylControlParameterization fromByte(int value)
      {
         return switch (value)
         {
            case 0 -> POLY_ZERO;
            case 1 -> POLY_ONE;
            case 2 -> POLY_TWO;
            default -> null;
         };
      }

      public int toByte()
      {
         return switch (this)
         {
            case POLY_ZERO -> 0;
            case POLY_ONE -> 1;
            case POLY_TWO -> 2;
            default -> -1;
         };
      }

   }
}
