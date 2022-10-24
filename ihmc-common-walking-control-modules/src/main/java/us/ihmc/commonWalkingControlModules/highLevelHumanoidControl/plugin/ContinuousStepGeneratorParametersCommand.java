package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParameters;
import us.ihmc.communication.controllerAPI.command.Command;

public class ContinuousStepGeneratorParametersCommand implements Command<ContinuousStepGeneratorParametersCommand, ContinuousStepGeneratorParametersMessage>
{
   private long sequenceId;
   private final ContinuousStepGeneratorParameters parameters = new ContinuousStepGeneratorParameters();

   public ContinuousStepGeneratorParametersCommand()
   {
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      parameters.clear();
   }

   @Override
   public void set(ContinuousStepGeneratorParametersCommand other)
   {
      sequenceId = other.sequenceId;
      parameters.set(other.parameters);
   }

   @Override
   public void setFromMessage(ContinuousStepGeneratorParametersMessage message)
   {
      sequenceId = message.getSequenceId();
      parameters.setNumberOfFootstepsToPlan(message.getNumberOfFootstepsToPlan());
      parameters.setNumberOfFixedFootsteps(message.getNumberOfFixedFootsteps());
      parameters.setSwingHeight(message.getSwingHeight());
      parameters.setSwingDuration(message.getSwingDuration());
      parameters.setTransferDuration(message.getTransferDuration());
      parameters.setMaxStepLength(message.getMaxStepLength());
      parameters.setDefaultStepWidth(message.getDefaultStepWidth());
      parameters.setMinStepWidth(message.getMinStepWidth());
      parameters.setMaxStepWidth(message.getMaxStepWidth());
      parameters.setTurnMaxAngleInward(message.getTurnMaxAngleInward());
      parameters.setTurnMaxAngleOutward(message.getTurnMaxAngleOutward());
   }

   public ContinuousStepGeneratorParameters getParameters()
   {
      return parameters;
   }

   @Override
   public Class<ContinuousStepGeneratorParametersMessage> getMessageClass()
   {
      return ContinuousStepGeneratorParametersMessage.class;
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
