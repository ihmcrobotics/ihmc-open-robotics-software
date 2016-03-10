package us.ihmc.darpaRoboticsChallenge.scriptEngine;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootstepDataListControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PauseWalkingControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisHeightTrajectoryControllerCommand;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptFileLoader;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptObject;
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
   private final ConcurrentLinkedQueue<ControllerCommand<?, ?>> controllerCommands;
   
   public ScriptBasedControllerCommandGenerator(ConcurrentLinkedQueue<ControllerCommand<?, ?>> controllerCommands)
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
         FootstepDataListControllerCommand command = new FootstepDataListControllerCommand();
         command.set(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof FootTrajectoryMessage)
      {
         FootTrajectoryMessage message = (FootTrajectoryMessage) scriptObject;
         FootTrajectoryControllerCommand command = new FootTrajectoryControllerCommand();
         command.set(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof HandTrajectoryMessage)
      {
         HandTrajectoryMessage message = (HandTrajectoryMessage) scriptObject;
         HandTrajectoryControllerCommand command = new HandTrajectoryControllerCommand();
         command.set(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof PelvisHeightTrajectoryMessage)
      {
         PelvisHeightTrajectoryMessage message = (PelvisHeightTrajectoryMessage) scriptObject;
         PelvisHeightTrajectoryControllerCommand command = new PelvisHeightTrajectoryControllerCommand();
         command.set(message);
         controllerCommands.add(command);
      }
      else if (scriptObject instanceof PauseWalkingMessage)
      {
         PauseWalkingMessage message = (PauseWalkingMessage) scriptObject;
         PauseWalkingControllerCommand command = new PauseWalkingControllerCommand();
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
