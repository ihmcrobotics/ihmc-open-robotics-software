package us.ihmc.steppr.hardware.controllers;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public interface StepprController
{
   public void setFullRobotModel(SDFFullRobotModel fullRobotModel);
   public void initialize(long timestamp);
   public void doControl(long timestamp);
   public YoVariableRegistry getYoVariableRegistry();
   
   public boolean turnOutputOn();
   
}
