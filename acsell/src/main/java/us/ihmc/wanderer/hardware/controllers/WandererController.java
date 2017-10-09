package us.ihmc.wanderer.hardware.controllers;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface WandererController
{
   public void setFullRobotModel(FullRobotModel fullRobotModel);
   public void initialize(long timestamp);
   public void doControl(long timestamp);
   public YoVariableRegistry getYoVariableRegistry();

   public boolean turnOutputOn();

}
