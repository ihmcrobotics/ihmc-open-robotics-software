package us.ihmc.exampleSimulations.buildingPendulum;

import com.vividsolutions.jts.index.bintree.Root;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.robotController.SimpleRobotController;

public class BuildingPendulumController extends SimpleRobotController
{
   private BuildingPendulumRobot robot;
   private double pendulumAngle;
   private double pendulumAngleSwitch;



   public BuildingPendulumController(BuildingPendulumRobot robot)
   {
      this.robot = robot;
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

      setPendulumAngles();

     boolean atCenter;
      if(pendulumAngle>pendulumAngleSwitch)
         atCenter = true;
      else
        atCenter = false;

     boolean shouldSwitchToLeft = activeSide == RobotSide.RIGHT && atCenter;
     boolean shouldSwitchToRight = activeSide == RobotSide.LEFT && atCenter;

      if (shouldSwitchToLeft)
      {
           robot.getPendulumAngle(RobotSide.RIGHT);

//         readRightPendulum;
//         computeLeft;
//         setLeftPensulum;
//         hideRightPensulum;
//         showLeftPensdulum;
         activeSide = RobotSide.LEFT;
      }
      else if (shouldSwitchToRight)
      {
        // ...;
      }

   }
}
