package us.ihmc.wanderer.hardware.controllers;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public interface WandererController
{
   public void setFullRobotModel(FullRobotModel fullRobotModel);
   public void initialize(long timestamp);
   public void doControl(long timestamp);
   public YoVariableRegistry getYoVariableRegistry();

   public boolean turnOutputOn();

}
