package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.EnumMap;

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
         OneDoFJoint[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();

         transitionControllerState = new SmoothTransitionControllerState("exitWalking", HighLevelControllerName.EXIT_WALKING, initialControllerState,
                                                                         finalControllerState, controlledJoints, controllerFactoryHelper.getHighLevelControllerParameters());
      }

      return transitionControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.EXIT_WALKING;
   }
}
