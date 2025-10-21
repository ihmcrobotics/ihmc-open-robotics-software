package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.CommandBlenderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointControlBlenderFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

import java.util.EnumMap;

public class SmoothTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private SmoothTransitionControllerState transitionControllerState; // The state that handles the transition
   private final String transitionName; // The name of the transition
   private final HighLevelControllerName transitionStateEnum; // The high level state enum of the transition state
   private final HighLevelControllerName startState; // The starting state (transitioning from here)
   private final HighLevelControllerName endState; // The ending state (transitioning to here)
   private final CommandBlenderFactory commandBlenderFactory;

   public SmoothTransitionControllerStateFactory(String transitionName,
                                                 HighLevelControllerName transitionStateEnum,
                                                 HighLevelControllerName startState,
                                                 HighLevelControllerName endState)
   {
      this(transitionName, transitionStateEnum, startState, endState, new JointControlBlenderFactory());
   }

   public SmoothTransitionControllerStateFactory(String transitionName,
                                                 HighLevelControllerName transitionStateEnum,
                                                 HighLevelControllerName startState,
                                                 HighLevelControllerName endState,
                                                 CommandBlenderFactory commandBlenderFactory)
   {
      this.transitionName = transitionName;
      this.transitionStateEnum = transitionStateEnum;
      this.startState = startState;
      this.endState = endState;
      this.commandBlenderFactory = commandBlenderFactory;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (transitionControllerState == null)
      {
         EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerFactoriesMap = controllerFactoryHelper.getControllerFactories();
         HighLevelControllerStateFactory initialControllerStateFactory = controllerFactoriesMap.get(startState);
         HighLevelControllerStateFactory finalControllerStateFactory = controllerFactoriesMap.get(endState);

         HighLevelControllerState initialControllerState = initialControllerStateFactory.getOrCreateControllerState(
               controllerFactoryHelper);
         HighLevelControllerState finalControllerState = finalControllerStateFactory.getOrCreateControllerState(
               controllerFactoryHelper);
         OneDoFJointBasics[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox()
                                                                       .getControlledOneDoFJoints();
         CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();

         transitionControllerState = new SmoothTransitionControllerState(transitionName,
                                                                         transitionStateEnum,
                                                                         initialControllerState,
                                                                         finalControllerState,
                                                                         controlledJoints,
                                                                         controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                         commandInputManager,
                                                                         commandBlenderFactory);
      }

      return transitionControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return transitionStateEnum;
   }
}
