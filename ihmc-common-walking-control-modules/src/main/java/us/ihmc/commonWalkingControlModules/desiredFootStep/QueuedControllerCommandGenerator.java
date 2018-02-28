package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager.StatusMessageListener;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HandTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;

public class QueuedControllerCommandGenerator implements Updatable
{
   private static final boolean DEBUG = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private boolean waitingForWalkingStatusToComplete = false;
   
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;

   private final List<Updatable> updatables = new ArrayList<>();

   private final ConcurrentLinkedQueue<Command<?, ?>> controllerCommands;
   
   public QueuedControllerCommandGenerator(ConcurrentLinkedQueue<Command<?, ?>> controllerCommands,
         CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
         WalkingControllerParameters walkingControllerParameters, CommonHumanoidReferenceFrames referenceFrames,
         SideDependentList<? extends ContactablePlaneBody> bipedFeet, double controlDT, boolean useHeadingAndVelocityScript, YoVariableRegistry parentRegistry)
   {
      this.controllerCommands = controllerCommands;
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;

      createFootstepStatusListener();

      parentRegistry.addChild(registry);
      
      updatables.add(this);
   }

   private void pollAndSubmitNextCommandIfReady()
   {
      if (waitingForWalkingStatusToComplete) return;

      if (controllerCommands.isEmpty())
      {
         waitingForWalkingStatusToComplete = false;
         return;
      }

      Command<?, ?> controllerCommand = controllerCommands.poll();
      if (controllerCommand == null) 
      {
         waitingForWalkingStatusToComplete = false;
         return;
      }

      if (DEBUG)
         System.out.println("Found a controller command!!!");
      if (controllerCommand instanceof FootstepDataListCommand)
      {
         FootstepDataListCommand footstepDataListControllerCommand = (FootstepDataListCommand) controllerCommand;
         commandInputManager.submitCommand(footstepDataListControllerCommand);
         waitingForWalkingStatusToComplete = true;
      }
      
      else if (controllerCommand instanceof ChestTrajectoryCommand)
      {
         ChestTrajectoryCommand chestTrajectoryControllerCommand = (ChestTrajectoryCommand) controllerCommand;
         commandInputManager.submitCommand(chestTrajectoryControllerCommand);
      }
      
      else if (controllerCommand instanceof FootTrajectoryCommand)
      {
         FootTrajectoryCommand footTrajectoryControllerCommand = (FootTrajectoryCommand) controllerCommand;
         commandInputManager.submitCommand(footTrajectoryControllerCommand);
      }
      
      else if (controllerCommand instanceof HandTrajectoryCommand)
      {
         HandTrajectoryCommand handTrajectoryControllerCommand = (HandTrajectoryCommand) controllerCommand;
         commandInputManager.submitCommand(handTrajectoryControllerCommand);
      }
      
      else if (controllerCommand instanceof PelvisHeightTrajectoryCommand)
      {
         PelvisHeightTrajectoryCommand pelvisHeightTrajectoryControllerCommand = (PelvisHeightTrajectoryCommand) controllerCommand;
         commandInputManager.submitCommand(pelvisHeightTrajectoryControllerCommand);
      }

      else if (controllerCommand instanceof PelvisTrajectoryCommand)
      {
         PelvisTrajectoryCommand pelvisTrajectoryControllerCommand = (PelvisTrajectoryCommand) controllerCommand;
         commandInputManager.submitCommand(pelvisTrajectoryControllerCommand);
      }
      
      else
      {
         System.err.println("QueuedControllerCommandGenerator: No plan for how to deal with commands of type " + controllerCommand.getClass().getSimpleName());
      }

   }
   
   public void addControllerCommand(Command<?, ?> controllerCommand)
   {
      controllerCommands.add(controllerCommand);
   }
   
   
   private void createFootstepStatusListener()
   {
      StatusMessageListener<FootstepStatusMessage> footstepStatusListener = new StatusMessageListener<FootstepStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(FootstepStatusMessage footstepStatus)
         {
            switch (FootstepStatus.fromByte(footstepStatus.footstepStatus))
            {
            case COMPLETED:
            {
               if (DEBUG)
               {
                  System.out.println("Footstep Completed!");
                  System.out.println("waitingForWalkingStatusToComplete = " + waitingForWalkingStatusToComplete);
               }

               pollAndSubmitNextCommandIfReady();
            }
            default:
               break;
            }
         }
      };
      statusOutputManager.attachStatusMessageListener(FootstepStatusMessage.class, footstepStatusListener);

      StatusMessageListener<WalkingStatusMessage> walkingStatusListener = new StatusMessageListener<WalkingStatusMessage>()
      {
         @Override
         public void receivedNewMessageStatus(WalkingStatusMessage walkingStatusListener)
         {
            switch (WalkingStatus.fromByte(walkingStatusListener.getWalkingStatus()))
            {
            case COMPLETED:
            {
               if (DEBUG)
               {
                  System.out.println("Walking Completed!");
                  System.out.println("waitingForWalkingStatusToComplete = " + waitingForWalkingStatusToComplete);
               }
               
               waitingForWalkingStatusToComplete = false;

            }
            case ABORT_REQUESTED:
            default:
               break;
            }
         }
      };
      statusOutputManager.attachStatusMessageListener(WalkingStatusMessage.class, walkingStatusListener);
   }

   public List<Updatable> getModulesToUpdate()
   {
      return updatables;
   }

   @Override
   public void update(double time)
   {
      if (!waitingForWalkingStatusToComplete)
      {
         this.pollAndSubmitNextCommandIfReady();
      }
   }
}
