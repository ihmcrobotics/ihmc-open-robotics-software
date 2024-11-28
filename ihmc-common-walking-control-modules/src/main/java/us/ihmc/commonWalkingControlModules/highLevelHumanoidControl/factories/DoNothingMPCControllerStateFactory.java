package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelMPCControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.DoNothingMPCControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.MPCHighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class DoNothingMPCControllerStateFactory implements MPCHighLevelControllerStateFactory
{
   private DoNothingMPCControllerState doNothingControllerState;

   @Override
   public MPCHighLevelControllerState getOrCreateControllerState(HighLevelMPCControllerFactoryHelper controllerFactoryHelper)
   {
      if (doNothingControllerState == null)
      {
         doNothingControllerState = new DoNothingMPCControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints(),
                                                                    controllerFactoryHelper.getHighLevelControllerParameters());
      }

      return doNothingControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }
}
