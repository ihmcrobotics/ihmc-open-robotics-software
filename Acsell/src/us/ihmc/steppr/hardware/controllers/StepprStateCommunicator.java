package us.ihmc.steppr.hardware.controllers;

import us.ihmc.SdfLoader.SDFBaseFullRobotModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprStateCommunicator implements StepprController
{

   public static void main(String[] args)
   {
      StepprSingleThreadedController.startController(new StepprStateCommunicator());
   }

   @Override
   public void setFullRobotModel(SDFBaseFullRobotModel fullRobotModel)
   {
      // Do nothing
   }

   @Override
   public void initialize(long timestamp)
   {
      // Do nothing
   }

   @Override
   public void doControl(long timestamp)
   {
      // Do nothing
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public boolean turnOutputOn()
   {
      return false;
   }

}
