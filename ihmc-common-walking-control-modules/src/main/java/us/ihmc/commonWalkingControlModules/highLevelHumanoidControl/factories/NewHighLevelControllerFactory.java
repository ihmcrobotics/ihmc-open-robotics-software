package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;

public interface NewHighLevelControllerFactory
{
   public abstract NewHighLevelControllerState createHighLevelController(HighLevelControlManagerFactory variousWalkingManagers,
                                                                       HighLevelHumanoidControllerToolbox controllerToolbox);

   public abstract boolean isTransitionToBehaviorRequested();
}
