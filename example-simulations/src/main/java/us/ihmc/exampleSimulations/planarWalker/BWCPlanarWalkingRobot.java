package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BWCPlanarWalkingRobot
{
   private final SideDependentList<SimPrismaticJoint> kneeJoints;

   private final SideDependentList<YoDouble> legLengths = new SideDependentList<YoDouble>();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   public BWCPlanarWalkingRobot(Robot robot)
   {
      robot.getFloatingRootJoint().setJointPosition(new Vector3D(0.0, 0.0, 0.75));

      kneeJoints = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         SimPrismaticJoint kneeJoint = (SimPrismaticJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.kneeNames.get(robotSide));
         kneeJoints.put(robotSide, kneeJoint);

         YoDouble legLength = new YoDouble(robotSide.getLowerCaseName() + "LegLength", registry);
         legLengths.put(robotSide, legLength);
      }
      kneeJoints.get(RobotSide.LEFT).setQ(0.25);
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public double getLegLength(RobotSide robotSide)
   {
      return legLengths.get(robotSide).getDoubleValue();
   }

   public SimPrismaticJoint getKneeJoint(RobotSide robotSide)
   {
      return kneeJoints.get(robotSide);
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         // update the current leg length
         double restingLegLength = (BWCPlanarWalkingRobotDefinition.thighLength + BWCPlanarWalkingRobotDefinition.shinLength) / 2.0;
         double currentLegLength = restingLegLength - kneeJoints.get(robotSide).getQ();

         legLengths.get(robotSide).set(currentLegLength);
      }
   }
}
