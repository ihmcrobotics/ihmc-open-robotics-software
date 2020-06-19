package us.ihmc.simpleWholeBodyWalking;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class SimpleWalkingControllerStateFactory implements HighLevelControllerStateFactory
{
   private SimpleWalkingControllerState walkingControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (walkingControllerState == null)
      {
         SimpleControlManagerFactory managerFactory = new SimpleControlManagerFactory(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getYoVariableRegistry());
         walkingControllerState = new SimpleWalkingControllerState(controllerFactoryHelper.getCommandInputManager(), controllerFactoryHelper.getStatusMessageOutputManager(),
                                                                   managerFactory, controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                   controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                   controllerFactoryHelper.getWalkingControllerParameters());
      }

      return walkingControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.WALKING;
   }
}
