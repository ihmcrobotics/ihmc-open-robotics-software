package us.ihmc.avatar.scriptCommandGenerator;

import controller_msgs.FootTrajectoryMessage;
import controller_msgs.FootstepDataListMessage;
import controller_msgs.HandTrajectoryMessage;
import controller_msgs.PauseWalkingMessage;
import controller_msgs.PelvisHeightTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

import java.util.concurrent.ConcurrentLinkedQueue;

public class ScriptBasedControllerCommandGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;
   private final FullHumanoidRobotModel fullRobotModel;
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver();

   public ScriptBasedControllerCommandGenerator(ConcurrentLinkedQueue<Command<?, ?>> controllerCommands, FullHumanoidRobotModel fullRobotModel)
   {
      this.controllerCommands = controllerCommands;
      this.fullRobotModel = fullRobotModel;
      referenceFrameHashCodeResolver.putAllFullRobotModelReferenceFrames(fullRobotModel);
   }

   public void runExerciseScript(ExerciseAndJUnitScript script, ReferenceFrame referenceFrame)
   {
      ExerciseAndJUnitScriptCommands.run(script, this, referenceFrame);
   }

   public void submitMessage(Object scriptObject)
   {
      if (scriptObject instanceof FootstepDataListMessage message)
      {
         FootstepDataListCommand command = new FootstepDataListCommand();
         command.setFromMessage(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof FootTrajectoryMessage message)
      {
         message.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         FootTrajectoryCommand command = new FootTrajectoryCommand();
         command.getSE3Trajectory().set(referenceFrameHashCodeResolver, message.getSe3Trajectory());
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof HandTrajectoryMessage message)
      {
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         message.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         HandTrajectoryCommand command = new HandTrajectoryCommand();
         command.getSE3Trajectory().set(referenceFrameHashCodeResolver, message.getSe3Trajectory());
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof PelvisHeightTrajectoryMessage message)
      {
         PelvisHeightTrajectoryCommand command = new PelvisHeightTrajectoryCommand();
         command.setFromMessage(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof PauseWalkingMessage message)
      {
         PauseWalkingCommand command = new PauseWalkingCommand();
         command.setFromMessage(message);
         controllerCommands.add(command);
      }
      else
      {
         System.err.println("ScriptBasedControllerCommandGenerator: Didn't process script object " + scriptObject);
      }
   }
}
