package us.ihmc.simulationConstructionSetTools.robotController;

import java.util.ArrayList;

public class SliderBoardControllerInitializer
{
   private static ArrayList<SliderBoardRobotController> robotControllers = new ArrayList<SliderBoardRobotController>();

   public static void register(SliderBoardRobotController sliderBoardRobotController)
   {
      robotControllers.add(sliderBoardRobotController);
   }

   public static void initialize()
   {
      for (SliderBoardRobotController sliderBoardRobotController : robotControllers)
      {
         sliderBoardRobotController.initializeSliders();
      }
   }
}
