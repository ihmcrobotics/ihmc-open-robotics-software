package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedScriptedFlatGroundWalkingTest;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

import java.util.ArrayList;
import java.util.List;

public class GenericQuadrupedScriptedFlatGroundWalkingTest extends QuadrupedScriptedFlatGroundWalkingTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }

   @Override
   @Test
   public void testScriptedFlatGroundWalking() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testScriptedFlatGroundWalking();
   }

   @Override
   @Test
   public void testScriptedTroublingSteps()
   {
      super.testScriptedTroublingSteps();
   }

   @Override
   public Point3D getFinalPlanarPosition()
   {
      return new Point3D(1.75, 0.0, 0.0);
   }

   @Override
   public List<QuadrupedTimedStepMessage> getSteps()
   {
      ArrayList<QuadrupedTimedStepMessage> steps = new ArrayList<>();
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(-0.550, -0.100, -0.012), 0.1, new TimeInterval(0.200, 0.530)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.550, 0.100, -0.012), 0.1, new TimeInterval(0.200, 0.530)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(-0.547, 0.100, -0.012), 0.1, new TimeInterval(0.630, 0.960)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(0.553, -0.100, -0.012), 0.1, new TimeInterval(0.630, 0.960)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(-0.544, -0.101, -0.012), 0.1, new TimeInterval(1.060, 1.390)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.556, 0.099, -0.012), 0.1, new TimeInterval(1.060, 1.390)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(-0.541, 0.096, -0.012), 0.1, new TimeInterval(1.490, 1.820)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(0.559, -0.104, -0.012), 0.1, new TimeInterval(1.490, 1.820)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(0.890, 0.096, -0.000), 0.1, new TimeInterval(2.710, 3.040)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.005, -0.104, -0.000), 0.1, new TimeInterval(2.925, 3.255)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.311, -0.100, -0.000), 0.1, new TimeInterval(3.140, 3.470)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(0.380, 0.109, -0.000), 0.1, new TimeInterval(3.355, 3.685)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(1.486, 0.118, -0.000), 0.1, new TimeInterval(3.570, 3.900)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(0.595, -0.073, -0.000), 0.1, new TimeInterval(3.785, 4.115)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(1.912, -0.066, -0.000), 0.1, new TimeInterval(4.000, 4.330)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_LEFT, new Point3D(1.023, 0.149, -0.000), 0.1, new TimeInterval(4.215, 4.545)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_LEFT, new Point3D(2.284, 0.137, -0.000), 0.1, new TimeInterval(4.430, 4.760)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.HIND_RIGHT, new Point3D(1.404, -0.083, -0.000), 0.1, new TimeInterval(4.645, 4.975)));
      steps.add(QuadrupedMessageTools
                      .createQuadrupedTimedStepMessage(RobotQuadrant.FRONT_RIGHT, new Point3D(2.708, -0.080, -0.000), 0.1, new TimeInterval(4.860, 5.190)));
      return steps;
   }
}
