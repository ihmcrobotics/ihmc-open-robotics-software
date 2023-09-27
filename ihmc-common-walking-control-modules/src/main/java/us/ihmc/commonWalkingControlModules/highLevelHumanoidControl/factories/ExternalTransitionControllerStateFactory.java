package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

import java.util.EnumMap;

public class ExternalTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private SmoothTransitionControllerState standTransitionControllerState;
   private final HighLevelControllerName startState, endState;

   public ExternalTransitionControllerStateFactory()
   {
      this(HighLevelControllerName.STAND_READY, HighLevelControllerName.EXTERNAL);
   }

   public ExternalTransitionControllerStateFactory(HighLevelControllerName startState, HighLevelControllerName endState)
   {
      this.startState = startState;
      this.endState = endState;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standTransitionControllerState == null)
      {
         EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerFactoriesMap = controllerFactoryHelper.getControllerFactories();
         HighLevelControllerStateFactory standReadyControllerStateFactory = controllerFactoriesMap.get(startState);
         HighLevelControllerStateFactory externalControlStateFactory = controllerFactoriesMap.get(endState);

         HighLevelControllerState standReadyControllerState = standReadyControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         HighLevelControllerState externalControllerState = externalControlStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         OneDoFJointBasics[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();

         standTransitionControllerState = new SmoothTransitionControllerState("toExternal",
                                                                              HighLevelControllerName.EXTERNAL_TRANSITION_STATE,
                                                                              standReadyControllerState,
                                                                              externalControllerState,
                                                                              controlledJoints,
                                                                              controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                              controllerFactoryHelper.getCommandInputManager());
      }

      return standTransitionControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.STAND_TRANSITION_STATE;
   }
}
