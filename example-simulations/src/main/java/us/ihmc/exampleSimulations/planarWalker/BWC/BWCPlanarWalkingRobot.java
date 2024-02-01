package us.ihmc.exampleSimulations.planarWalker.BWC;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BWCPlanarWalkingRobot
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<SimPrismaticJoint> kneeJoints = new SideDependentList<>();

   private final SideDependentList<YoDouble>  legLengths = new SideDependentList<>();

   private final double restingLegLength;

   public BWCPlanarWalkingRobot(Robot robot)
   {
      robot.getFloatingRootJoint().setJointPosition(new Point3D(0.0, 0.0, 0.6));

      restingLegLength = (BWCPlanarWalkingRobotDefinition.shinLength + BWCPlanarWalkingRobotDefinition.thighLength) / 2;

      kneeJoints.put(RobotSide.LEFT, (SimPrismaticJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.leftKneeName));
      kneeJoints.put(RobotSide.RIGHT, (SimPrismaticJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.rightKneeName));

      for (RobotSide robotSide : RobotSide.values())
      {
         legLengths.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "legLength", registry));
      }
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         double currentLegLength = restingLegLength - kneeJoints.get(robotSide).getQ();
         legLengths.get(robotSide).set(currentLegLength);
      }
   }

   public SimPrismaticJoint getKneeJoint(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide);
   }

   public double getLegLength(RobotSide robotSide)
   {
      return legLengths.get(robotSide).getDoubleValue();
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}

