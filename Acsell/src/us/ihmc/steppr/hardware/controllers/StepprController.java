package us.ihmc.steppr.hardware.controllers;

import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public interface StepprController
{
   public void setFullRobotModel(FullRobotModel fullRobotModel);
   public void initialize();
   public void doControl();
   public YoVariableRegistry getYoVariableRegistry();
   
}
