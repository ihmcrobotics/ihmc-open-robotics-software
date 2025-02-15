package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class ScriptBasedControllerCommandGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ConcurrentLinkedQueue<ScriptObject> scriptObjects = new ConcurrentLinkedQueue<ScriptObject>();
   private final ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;
   private final FullHumanoidRobotModel fullRobotModel;
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver();

   public ScriptBasedControllerCommandGenerator(ConcurrentLinkedQueue<Command<?, ?>> controllerCommands, FullHumanoidRobotModel fullRobotModel)
   {
      this.controllerCommands = controllerCommands;
      this.fullRobotModel = fullRobotModel;
      referenceFrameHashCodeResolver.putAllFullRobotModelReferenceFrames(fullRobotModel);
   }

   public void loadScriptFile(Path scriptFilePath, ReferenceFrame referenceFrame)
   {
      ScriptFileLoader scriptFileLoader;
      try
      {
         scriptFileLoader = new ScriptFileLoader(scriptFilePath);

         RigidBodyTransform transformFromReferenceFrameToWorldFrame = referenceFrame.getTransformToDesiredFrame(worldFrame);
         ArrayList<ScriptObject> scriptObjectsList = scriptFileLoader.readIntoList(transformFromReferenceFrameToWorldFrame);
         scriptObjects.addAll(scriptObjectsList);
         convertFromScriptObjectsToControllerCommands();
      }
      catch (IOException e)
      {
         System.err.println("Could not load script file " + scriptFilePath);
      }
   }

   public void loadScriptFile(InputStream scriptInputStream, ReferenceFrame referenceFrame)
   {
      ScriptFileLoader scriptFileLoader;
      try
      {
         scriptFileLoader = new ScriptFileLoader(scriptInputStream);

         RigidBodyTransform transformFromReferenceFrameToWorldFrame = referenceFrame.getTransformToDesiredFrame(worldFrame);
         ArrayList<ScriptObject> scriptObjectsList = scriptFileLoader.readIntoList(transformFromReferenceFrameToWorldFrame);
         scriptObjects.addAll(scriptObjectsList);
         convertFromScriptObjectsToControllerCommands();
      }
      catch (IOException e)
      {
         System.err.println("Could not load script file " + scriptInputStream);
      }

   }

   private void convertFromScriptObjectsToControllerCommands()
   {
      while(!scriptObjects.isEmpty())
      {
      ScriptObject nextObject = scriptObjects.poll();
      Object scriptObject = nextObject.getScriptObject();

      if (scriptObject instanceof FootstepDataListMessage)
      {
         FootstepDataListMessage message = (FootstepDataListMessage) scriptObject;
         FootstepDataListCommand command = new FootstepDataListCommand();
         command.setFromMessage(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof FootTrajectoryMessage)
      {
         FootTrajectoryMessage message = (FootTrajectoryMessage) scriptObject;
         message.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(worldFrame));
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         FootTrajectoryCommand command = new FootTrajectoryCommand();
         command.getSE3Trajectory().set(referenceFrameHashCodeResolver, message.getSe3Trajectory());
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof HandTrajectoryMessage)
      {
         ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
         HandTrajectoryMessage message = (HandTrajectoryMessage) scriptObject;
         message.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chestFrame));
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
         HandTrajectoryCommand command = new HandTrajectoryCommand();
         command.getSE3Trajectory().set(referenceFrameHashCodeResolver, message.getSe3Trajectory());
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof PelvisHeightTrajectoryMessage)
      {
         PelvisHeightTrajectoryMessage message = (PelvisHeightTrajectoryMessage) scriptObject;
         PelvisHeightTrajectoryCommand command = new PelvisHeightTrajectoryCommand();
         command.setFromMessage(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof PauseWalkingMessage)
      {
         PauseWalkingMessage message = (PauseWalkingMessage) scriptObject;
         PauseWalkingCommand command = new PauseWalkingCommand();
         command.setFromMessage(message);
         controllerCommands.add(command);
      }


//      else if (scriptObject instanceof ArmTrajectoryMessage)
//      {
//         ArmTrajectoryMessage armTrajectoryMessage = (ArmTrajectoryMessage) scriptObject;
//         armTrajectoryMessageSubscriber.receivedPacket(armTrajectoryMessage);
//
//         setupTimesForNewScriptEvent(armTrajectoryMessage.getTrajectoryTime());
//      }


      else
      {
         System.err.println("ScriptBasedControllerCommandGenerator: Didn't process script object " + nextObject);
      }
   }

   }





}
