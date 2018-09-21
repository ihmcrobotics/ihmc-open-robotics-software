package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

import java.util.EnumMap;

public class StandTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private SmoothTransitionControllerState standTransitionControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standTransitionControllerState == null)
      {
         EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerFactoriesMap = controllerFactoryHelper.getControllerFactories();
         HighLevelControllerStateFactory standReadyControllerStateFactory = controllerFactoriesMap.get(HighLevelControllerName.STAND_READY);
         HighLevelControllerStateFactory walkingControllerStateFactory = controllerFactoriesMap.get(HighLevelControllerName.WALKING);

         HighLevelControllerState standReadyControllerState = standReadyControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         HighLevelControllerState walkingControllerState = walkingControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         OneDoFJoint[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();

         standTransitionControllerState = new SmoothTransitionControllerState("toWalking", HighLevelControllerName.STAND_TRANSITION_STATE,
                                                                              standReadyControllerState, walkingControllerState,
                                                                              controlledJoints, controllerFactoryHelper.getHighLevelControllerParameters());
      }

      return standTransitionControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.STAND_TRANSITION_STATE;
   }
}
