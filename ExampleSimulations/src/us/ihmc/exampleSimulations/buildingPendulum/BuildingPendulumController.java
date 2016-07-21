package us.ihmc.exampleSimulations.buildingPendulum;

import com.vividsolutions.jts.index.bintree.Root;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class BuildingPendulumController extends SimpleRobotController
{
   private BuildingPendulumRobot robot;
   private double pendulumAngle;
   private double pendulumAngleSwitch;

   private final EnumYoVariable<RobotSide> yoActiveSide = new EnumYoVariable<>("ActiveSide", registry, RobotSide.class);

   public BuildingPendulumController(BuildingPendulumRobot robot)
   {
      this.robot = robot;
      activeSide = RobotSide.RIGHT; // figure out which side to start with
   }

   RobotSide activeSide;
   public void setPendulumAngles()
   {
      pendulumAngle =robot.getPendulumAngle(activeSide);
      pendulumAngleSwitch = robot.getSwitchAngle(activeSide);
   }
   public  void doControl()
   {

      // compute torques, decide which pendulum active ...
     // robot.pendulumJoint1.
      setPendulumAngles();

     boolean atCenter= pendulumAngle>pendulumAngleSwitch;
//      if(pendulumAngle>pendulumAngleSwitch)
//         atCenter = true;
//      else
//        atCenter = false;

     boolean shouldSwitchToLeft = activeSide == RobotSide.RIGHT && atCenter;
      System.out.println("left: "+shouldSwitchToLeft);

     boolean shouldSwitchToRight = activeSide == RobotSide.LEFT && atCenter;
      System.out.println("right: "+shouldSwitchToRight);
      double mass = BuildingPendulumRobot.mass;

      if (shouldSwitchToLeft)
      {
           robot.getPendulumAngle(RobotSide.RIGHT);
         double value = robot.pendulumJoint1.getQ().getDoubleValue();


//         readRightPendulum;

         double value2 = robot.pendulumJoint1.getQ().getDoubleValue();
         robot.pendulumJoint2.getQ().set(value+value2);

//         computeLeft;
//         setLeftPensulum;


//         hideRightPensulum;
//         showLeftPensdulum;
         activeSide = RobotSide.LEFT;
      }
      else if (shouldSwitchToRight)
      {
         robot.getPendulumAngle(RobotSide.LEFT);

         //         readRightPendulum;
         //         computeLeft;
         //         setLeftPensulum;
         //         hideRightPensulum;
         //         showLeftPensdulum;
         activeSide = RobotSide.RIGHT;
      }

      yoActiveSide.set(activeSide);
   }
}
