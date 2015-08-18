package us.ihmc.steppr.hardware.controllers;

import us.ihmc.SdfLoader.SDFBaseFullRobotModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public interface StepprController
{
   public void setFullRobotModel(SDFBaseFullRobotModel fullRobotModel);
   public void initialize(long timestamp);
   public void doControl(long timestamp);
   public YoVariableRegistry getYoVariableRegistry();
   
   public boolean turnOutputOn();
   
}
