package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class ExitWalkingTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private SmoothTransitionControllerState transitionControllerState;
   private final HighLevelControllerName targetState;

   public ExitWalkingTransitionControllerStateFactory(HighLevelControllerName targetState)
   {
      this.targetState = targetState;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (transitionControllerState == null)
      {
         EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerFactoriesMap = controllerFactoryHelper.getControllerFactories();
         HighLevelControllerStateFactory initialControllerStateFactory = controllerFactoriesMap.get(HighLevelControllerName.WALKING);
         HighLevelControllerStateFactory finalControllerStateFactory = controllerFactoriesMap.get(targetState);

         HighLevelControllerState initialControllerState = initialControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         HighLevelControllerState finalControllerState = finalControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         OneDoFJointBasics[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();
         CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();

         transitionControllerState = new SmoothTransitionControllerState("exitWalking",
                                                                         HighLevelControllerName.EXIT_WALKING,
                                                                         initialControllerState,
                                                                         finalControllerState,
                                                                         controlledJoints,
                                                                         controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                         commandInputManager);
      }

      return transitionControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.EXIT_WALKING;
   }
}
