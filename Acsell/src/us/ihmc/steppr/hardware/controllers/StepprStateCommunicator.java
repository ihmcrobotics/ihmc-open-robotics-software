package us.ihmc.steppr.hardware.controllers;

import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class StepprStateCommunicator implements StepprController
{

   public static void main(String[] args)
   {
      StepprSingleThreadedController.startController(new StepprStateCommunicator());
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      // Do nothing
   }

   @Override
   public void initialize()
   {
      // Do nothing
   }

   @Override
   public void doControl()
   {
      // Do nothing
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

}
