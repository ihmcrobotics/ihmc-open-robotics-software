package us.ihmc.moonwalking.models.aaronhopper;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class AaronHopperController implements RobotController
{
   YoVariable q_x, q_z, q_pitch, tau_hip, tau_knee;
   AaronHopperRobot Robot;

   public AaronHopperController(AaronHopperRobot Robot)
   {
      this.Robot = Robot;
   }

   public void doControl()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      // TODO Auto-generated method stub
      return null;
   }

   private void InitVars()
   {
      q_x = Robot.getVariable("q_x");
      q_z = Robot.getVariable("q_z");
      q_pitch = Robot.getVariable("q_pitch");
      tau_hip = Robot.getVariable("tau_hip");
      tau_knee = Robot.getVariable("tau_knee");
   }

   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   public String getName()
   {
      // TODO Auto-generated method stub
      return null;
   }

   public String getDescription()
   {
      // TODO Auto-generated method stub
      return null;
   }
}
