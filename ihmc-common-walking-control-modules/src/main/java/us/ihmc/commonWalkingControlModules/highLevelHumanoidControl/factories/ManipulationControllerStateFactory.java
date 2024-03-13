package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ManipulationControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

public class ManipulationControllerStateFactory implements HighLevelControllerStateFactory
{
   private ManipulationControllerState manipulationControllerState;
   private final HumanoidJointNameMap jointNameMap;

   public ManipulationControllerStateFactory(HumanoidJointNameMap jointNameMap)
   {
      this.jointNameMap = jointNameMap;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (manipulationControllerState == null)
      {
         manipulationControllerState = new ManipulationControllerState(controllerFactoryHelper.getCommandInputManager(),
                                                                       controllerFactoryHelper.getStatusMessageOutputManager(),
                                                                       controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlDT(),
                                                                       jointNameMap,
                                                                       controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                       controllerFactoryHelper.getWalkingControllerParameters(),
                                                                       controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel(),
                                                                       null,
                                                                       controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getYoTime(),
                                                                       controllerFactoryHelper.getHighLevelHumanoidControllerToolbox()
                                                                                              .getYoGraphicsListRegistry());
      }

      return manipulationControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.WALKING;
   }
}
