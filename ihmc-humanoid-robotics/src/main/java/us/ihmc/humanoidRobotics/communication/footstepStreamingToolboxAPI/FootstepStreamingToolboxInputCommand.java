package us.ihmc.humanoidRobotics.communication.footstepStreamingToolboxAPI;

import toolbox_msgs.FootstepStreamingToolboxInputMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.List;

public class FootstepStreamingToolboxInputCommand implements Command<FootstepStreamingToolboxInputCommand, FootstepStreamingToolboxInputMessage>
{
   private long sequenceId;
   private double robotStepDuration;
   private double robotSwingDuration;
   private double robotElapsedTimeCurrentStep;
   private RobotSide robotSwingSide;
   private boolean isRobotSwingFootLanding;
   private final RecyclingArrayList<FootstepStreamingToolboxSideCommand> inputs = new RecyclingArrayList<>(FootstepStreamingToolboxSideCommand::new);

   @Override
   public void clear()
   {
      sequenceId = 0;
      inputs.clear();
      robotStepDuration = 0.0;
      robotSwingDuration  = 0.0;
      robotElapsedTimeCurrentStep = 0.0;
      robotSwingSide = RobotSide.LEFT;
      isRobotSwingFootLanding = false;
   }

   @Override
   public void set(FootstepStreamingToolboxInputCommand other)
   {
      sequenceId = other.sequenceId;
      inputs.clear();
      for (int i = 0; i < other.inputs.size(); i++)
         inputs.add().set(other.inputs.get(i));
      robotStepDuration = other.robotStepDuration;
      robotSwingDuration = other.robotSwingDuration;
      robotElapsedTimeCurrentStep = other.robotElapsedTimeCurrentStep;
      robotSwingSide = other.robotSwingSide;
      isRobotSwingFootLanding = other.isRobotSwingFootLanding;
   }

   @Override
   public void setFromMessage(FootstepStreamingToolboxInputMessage message)
   {
      sequenceId = message.getSequenceId();
      inputs.clear();
      for (int i = 0; i < message.getSide().size(); i++)
         inputs.add().setFromMessage(message.getSide().get(i));
      robotStepDuration = message.getRobotStepDuration();
      robotSwingDuration = message.getRobotSwingDuration();
      robotElapsedTimeCurrentStep = message.getRobotStepElapsedTime();
      robotSwingSide = RobotSide.fromByte(message.getRobotSwingSide());
      isRobotSwingFootLanding = message.getIsRobotSwingFootLanding();
   }

   public void removeInput(int index)
   {
      inputs.remove(index);
   }

   public void removeInput(FootstepStreamingToolboxSideCommand input)
   {
      inputs.remove(input);
   }

   public int getNumberOfInputs()
   {
      return inputs.size();
   }

   public FootstepStreamingToolboxSideCommand getInput(int index)
   {
      return inputs.get(index);
   }

   public List<FootstepStreamingToolboxSideCommand> getInputs()
   {
      return inputs;
   }

   public boolean hasInputFor(RobotSide side)
   {
      return getInputFor(side) != null;
   }

   public FootstepStreamingToolboxSideCommand getInputFor(RobotSide side)
   {
      for (int i = 0; i < inputs.size(); i++)
      {
         if (inputs.get(i).getSide() == side)
            return inputs.get(i);
      }
      return null;
   }

   public double getRobotStepDuration()
   {
      return robotStepDuration;
   }

   public double getRobotSwingDuration()
   {
      return robotSwingDuration;
   }

   public double getRobotElapsedTimeCurrentStep()
   {
      return robotElapsedTimeCurrentStep;
   }

   public RobotSide getRobotSwingSide()
   {
      return robotSwingSide;
   }

   public boolean isRobotSwingFootLanding()
   {
      return isRobotSwingFootLanding;
   }

   @Override
   public Class<FootstepStreamingToolboxInputMessage> getMessageClass()
   {
      return FootstepStreamingToolboxInputMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      for (int i = 0; i < inputs.size(); i++)
      {
         if (!inputs.get(i).isCommandValid())
            return false;
      }
      return !Double.isNaN(robotStepDuration) && !Double.isNaN(robotElapsedTimeCurrentStep);
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
