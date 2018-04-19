package us.ihmc.quadrupedRobotics.planning;

import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.EuclideanTrajectoryPointMessagePubSubType;
import controller_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import org.junit.After;
import org.junit.Before;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.List;

public abstract class QuadrupedBodyPathPlanTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      conductor.concludeTesting();
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void testSimpleBodyPathPlan()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180);

      EuclideanTrajectoryPointMessage point1 = new EuclideanTrajectoryPointMessage();
      point1.setTime(2.0);
      point1.position_.set(0.5, 0.0, 0.0);

      EuclideanTrajectoryPointMessage point2 = new EuclideanTrajectoryPointMessage();
      point2.setTime(4.0);
      point2.position_.set(0.5, 0.5, 0.0);

      EuclideanTrajectoryPointMessage point3 = new EuclideanTrajectoryPointMessage();
      point3.setTime(6.0);
      point3.position_.set(0.0, 0.5, 0.0);

      EuclideanTrajectoryPointMessage point4 = new EuclideanTrajectoryPointMessage();
      point4.setTime(8.0);
      point4.position_.set(0.0, 0.0, 0.0);

      QuadrupedBodyPathPlanMessage bodyPathPlanMessage = new QuadrupedBodyPathPlanMessage();
      bodyPathPlanMessage.setIsExpressedInAbsoluteTime(false);
      Object<EuclideanTrajectoryPointMessage> bodyPathPoints = new Object<> (4, EuclideanTrajectoryPointMessage.class, new EuclideanTrajectoryPointMessagePubSubType());
      bodyPathPoints.add().set(point1);
      bodyPathPoints.add().set(point2);
      bodyPathPoints.add().set(point3);
      bodyPathPoints.add().set(point4);
      bodyPathPlanMessage.body_path_points_ = bodyPathPoints;

      stepTeleopManager.handleBodyPathPlanMessage(bodyPathPlanMessage);

      conductor.addTimeLimit(variables.getYoTime(), 15.0);
      conductor.simulate();
   }

}
