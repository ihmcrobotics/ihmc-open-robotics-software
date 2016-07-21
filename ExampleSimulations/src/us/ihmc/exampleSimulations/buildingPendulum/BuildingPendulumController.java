package us.ihmc.exampleSimulations.buildingPendulum;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class BuildingPendulumController extends SimpleRobotController
{
   private final BuildingPendulumRobot robot;
   private final BooleanYoVariable atCenter = new BooleanYoVariable("AtCenter", registry);

   private RobotSide activeSide;
   private final EnumYoVariable<RobotSide> yoActiveSide = new EnumYoVariable<>("ActiveSide", registry, RobotSide.class);

   private double pendulumAngle;
   private double pendulumAngleSwitch;

   public BuildingPendulumController(BuildingPendulumRobot robot)
   {
      this.robot = robot;
      activeSide = RobotSide.LEFT; // figure out which side to start with TODO
   }

   public void setPendulumAngles()
   {
      pendulumAngle =robot.getPendulumAngle(activeSide);
      pendulumAngleSwitch = robot.getSwitchAngle(activeSide);
   }

   public  void doControl()
   {
      setPendulumAngles();

      boolean atCenter;
      if (activeSide == RobotSide.LEFT)
         atCenter = pendulumAngle > pendulumAngleSwitch;
      else
         atCenter = pendulumAngle < pendulumAngleSwitch;

      if (atCenter)
      {
         activeSide = activeSide.getOppositeSide();
         robot.setPendulumAngle(activeSide, robot.getSwitchAngle(activeSide));

         // test velocity: TODO
//         double v = activeSide == RobotSide.LEFT ? -0.5 : 0.5;
         double v;
         if (activeSide == RobotSide.LEFT)
            v = -0.5;
         else
            v = 0.5;

         robot.setPendulumVelocity(activeSide, v);
      }

      // set the inactive pendulum to stay at the switching position
      robot.setPendulumAngle(activeSide.getOppositeSide(), robot.getSwitchAngle(activeSide.getOppositeSide()));
      robot.setPendulumVelocity(activeSide.getOppositeSide(), 0.0);

      // set yoVariables for debugging in SCS
      this.atCenter.set(atCenter);
      yoActiveSide.set(activeSide);
   }
}
