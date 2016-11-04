package us.ihmc.humanoidBehaviors.behaviors.scripts.engine;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PauseWalkingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ScriptBasedControllerCommandGenerator
{
   private final ConcurrentLinkedQueue<ScriptObject> scriptObjects = new ConcurrentLinkedQueue<ScriptObject>();
   private final ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;
   
   public ScriptBasedControllerCommandGenerator(ConcurrentLinkedQueue<Command<?, ?>> controllerCommands)
   {
      this.controllerCommands = controllerCommands;
   }

   public void loadScriptFile(String scriptFilename, ReferenceFrame referenceFrame)
   {
      ScriptFileLoader scriptFileLoader;
      try
      {
         scriptFileLoader = new ScriptFileLoader(scriptFilename);
         
         RigidBodyTransform transformFromReferenceFrameToWorldFrame = referenceFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
         ArrayList<ScriptObject> scriptObjectsList = scriptFileLoader.readIntoList(transformFromReferenceFrameToWorldFrame);
         scriptObjects.addAll(scriptObjectsList);
         convertFromScriptObjectsToControllerCommands();
      }
      catch (IOException e)
      {
         System.err.println("Could not load script file " + scriptFilename);
      }            
   }
   
   public void loadScriptFile(InputStream scriptInputStream, ReferenceFrame referenceFrame)
   {
      ScriptFileLoader scriptFileLoader;
      try
      {
         scriptFileLoader = new ScriptFileLoader(scriptInputStream);
         
         RigidBodyTransform transformFromReferenceFrameToWorldFrame = referenceFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame());
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
         command.set(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof FootTrajectoryMessage)
      {
         FootTrajectoryMessage message = (FootTrajectoryMessage) scriptObject;
         FootTrajectoryCommand command = new FootTrajectoryCommand();
         command.set(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof HandTrajectoryMessage)
      {
         HandTrajectoryMessage message = (HandTrajectoryMessage) scriptObject;
         HandTrajectoryCommand command = new HandTrajectoryCommand();
         command.set(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof PelvisHeightTrajectoryMessage)
      {
         PelvisHeightTrajectoryMessage message = (PelvisHeightTrajectoryMessage) scriptObject;
         PelvisHeightTrajectoryCommand command = new PelvisHeightTrajectoryCommand();
         command.set(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof PauseWalkingMessage)
      {
         PauseWalkingMessage message = (PauseWalkingMessage) scriptObject;
         PauseWalkingCommand command = new PauseWalkingCommand();
         command.set(message);
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
