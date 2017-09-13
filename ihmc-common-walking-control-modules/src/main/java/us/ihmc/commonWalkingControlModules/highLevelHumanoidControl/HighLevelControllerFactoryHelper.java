package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;
import java.util.concurrent.atomic.AtomicReference;

public class HighLevelControllerFactoryHelper
{
   private EnumMap<HighLevelControllerState, HighLevelControllerStateFactory> controllerFactories;
   private HighLevelHumanoidControllerToolbox controllerToolbox;
   private HighLevelControlManagerFactory managerFactory;
   private HighLevelControllerParameters highLevelControllerParameters;
   private WalkingControllerParameters walkingControllerParameters;
   private ICPTrajectoryPlannerParameters icpPlannerParameters;
   private CommandInputManager commandInputManager;
   private StatusMessageOutputManager statusMessageOutputManager;
   private LowLevelOneDoFJointDesiredDataHolderList lowLevelControllerOutput;
   private YoEnum<HighLevelControllerState> requestedHighLevelControllerState;
   private AtomicReference<HighLevelControllerState> fallbackControllerForFailure;

   public void setLowLevelControllerOutput(LowLevelOneDoFJointDesiredDataHolderList lowLevelControllerOutput)
   {
      this.lowLevelControllerOutput = lowLevelControllerOutput;
   }

   public void setControllerFactories(EnumMap<HighLevelControllerState, HighLevelControllerStateFactory> controllerFactories)
   {
      this.controllerFactories = controllerFactories;
   }

   public void setHighLevelHumanoidControllerToolbox(HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      this.controllerToolbox = controllerToolbox;
   }

   public void setParameters(HighLevelControllerParameters highLevelControllerParameters, WalkingControllerParameters walkingControllerParameters,
                             ICPTrajectoryPlannerParameters icpPlannerParameters)
   {
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      this.icpPlannerParameters = icpPlannerParameters;
   }

   public void setHighLevelControlManagerFactory(HighLevelControlManagerFactory managerFactory)
   {
      this.managerFactory = managerFactory;
   }

   public void setCommandInputManager(CommandInputManager commandInputManager)
   {
      this.commandInputManager = commandInputManager;
   }

   public void setStatusMessageOutputManager(StatusMessageOutputManager statusMessageOutputManager)
   {
      this.statusMessageOutputManager = statusMessageOutputManager;
   }

   public void setRequestedHighLevelControllerState(YoEnum<HighLevelControllerState> requestedHighLevelControllerState)
   {
      this.requestedHighLevelControllerState = requestedHighLevelControllerState;
   }

   public void setFallbackControllerForFailure(AtomicReference<HighLevelControllerState> fallbackControllerForFailure)
   {
      this.fallbackControllerForFailure = fallbackControllerForFailure;
   }

   public EnumMap<HighLevelControllerState, HighLevelControllerStateFactory> getControllerFactories()
   {
      return controllerFactories;
   }

   public HighLevelHumanoidControllerToolbox getHighLevelHumanoidControllerToolbox()
   {
      return controllerToolbox;
   }

   public HighLevelControlManagerFactory getManagerFactory()
   {
      return managerFactory;
   }

   public HighLevelControllerParameters getHighLevelControllerParameters()
   {
      return highLevelControllerParameters;
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   public ICPTrajectoryPlannerParameters getIcpPlannerParameters()
   {
      return icpPlannerParameters;
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public StatusMessageOutputManager getStatusMessageOutputManager()
   {
      return statusMessageOutputManager;
   }

   public LowLevelOneDoFJointDesiredDataHolderList getLowLevelControllerOutput()
   {
      return lowLevelControllerOutput;
   }

   public YoEnum<HighLevelControllerState> getRequestedHighLevelControllerState()
   {
      return requestedHighLevelControllerState;
   }

   public AtomicReference<HighLevelControllerState> getFallbackControllerForFailure()
   {
      return fallbackControllerForFailure;
   }
}
