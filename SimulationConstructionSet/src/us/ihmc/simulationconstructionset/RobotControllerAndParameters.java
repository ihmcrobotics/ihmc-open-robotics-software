package us.ihmc.simulationconstructionset;

import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.robotics.robotController.RobotController;

public class RobotControllerAndParameters
{
   protected final RobotController controller;
   protected final int simulationTicksPerControlTick;
   protected final YoInteger ticks_till_control;

   public RobotControllerAndParameters(RobotController controller, int simulationTicksPerControlTick)
   {
      this.controller = controller;
      this.simulationTicksPerControlTick = simulationTicksPerControlTick;
      this.ticks_till_control = new YoInteger("ticks_till_control_" + controller.getName(), controller.getYoVariableRegistry());
      ticks_till_control.set(simulationTicksPerControlTick);
   }

   public RobotController getController()
   {
      return controller;
   }

   public int getSimulationTicksPerControlTick()
   {
      return simulationTicksPerControlTick;
   }
}
