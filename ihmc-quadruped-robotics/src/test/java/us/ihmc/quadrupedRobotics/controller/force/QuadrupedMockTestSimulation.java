package us.ihmc.quadrupedRobotics.controller.force;

import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;

import java.io.IOException;

/**
 * Simple class for running a sim that has the same registry structure as a test without being a test
 */
public abstract class QuadrupedMockTestSimulation implements QuadrupedMultiRobotTestInterface
{
   public QuadrupedMockTestSimulation() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      GoalOrientedTestConductor conductor = quadrupedTestFactory.createTestConductor();
      conductor.getScs().startOnAThread();
   }
}
