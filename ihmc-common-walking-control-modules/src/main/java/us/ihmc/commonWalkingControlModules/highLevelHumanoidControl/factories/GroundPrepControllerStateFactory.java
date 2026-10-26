package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.GroundPrepControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class GroundPrepControllerStateFactory implements HighLevelControllerStateFactory
{
   private GroundPrepControllerState groundPrepControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (groundPrepControllerState == null)
      {
         groundPrepControllerState = new GroundPrepControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints(),
                                                                   controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                   controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getYoTime());
      }

      return groundPrepControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.GROUND_PREP_STATE;
   }
}
