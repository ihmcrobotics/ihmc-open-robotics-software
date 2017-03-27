package us.ihmc.exampleSimulations.buildingPendulum;

import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class BuildingPendulumController extends SimpleRobotController
{
   public static String variableName = "MySliderBoardTest";

   private final BuildingPendulumRobot robot;
   private final BooleanYoVariable atCenter = new BooleanYoVariable("AtCenter", registry);

   private RobotSide activeSide;
   private final EnumYoVariable<RobotSide> yoActiveSide = new EnumYoVariable<>("ActiveSide", registry, RobotSide.class);

   private double pendulumAngle;
   private double pendulumAngleSwitch;
   private double angularChange = 2*Math.asin(BuildingPendulumRobot.distance/(2*BuildingPendulumRobot.length));


   private final DoubleYoVariable mySliderBoardTest = new DoubleYoVariable(variableName, registry);

   public BuildingPendulumController(BuildingPendulumRobot robot)
   {
      this.robot = robot;

      activeSide = RobotSide.LEFT;

//      if(robot.getPendulumAngle(RobotSide.LEFT)> robot.getPendulumAngle(RobotSide.RIGHT))
//         activeSide = RobotSide.LEFT;
//      else if(robot.getPendulumAngle(RobotSide.LEFT)< robot.getPendulumAngle(RobotSide.RIGHT))
//         activeSide = RobotSide.RIGHT;

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
         atCenter = (pendulumAngle > pendulumAngleSwitch);
      else
         atCenter = (pendulumAngle < pendulumAngleSwitch);

      if (atCenter)
      {

         activeSide = activeSide.getOppositeSide();
         robot.setPendulumAngle(activeSide, robot.getSwitchAngle(activeSide));
         
         double velocity = (robot.getPendulumVelocity(activeSide.getOppositeSide())*Math.cos(angularChange));

         robot.setPendulumVelocity(activeSide, velocity);
      }

      // set the inactive pendulum to stay at the switching position
      robot.setPendulumAngle(activeSide.getOppositeSide(), robot.getSwitchAngle(activeSide.getOppositeSide()));
      robot.setPendulumVelocity(activeSide.getOppositeSide(), 0.0);

      // set yoVariables for debugging in SCS
      this.atCenter.set(atCenter);
      yoActiveSide.set(activeSide);

      // try out new slider variable
      if (mySliderBoardTest.getDoubleValue() <= -0.5)
         System.out.println("small value...");
   }
}
