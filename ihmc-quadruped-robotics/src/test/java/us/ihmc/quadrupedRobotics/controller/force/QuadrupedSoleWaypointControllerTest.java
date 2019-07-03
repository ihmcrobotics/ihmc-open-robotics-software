package us.ihmc.quadrupedRobotics.controller.force;

import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.SoleTrajectoryMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.QuadrupedControllerAPIDefinition;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

import static us.ihmc.robotics.Assert.assertEquals;

public abstract class QuadrupedSoleWaypointControllerTest implements QuadrupedMultiRobotTestInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;
   private IHMCROS2Publisher<SoleTrajectoryMessage> soleTrajectoryPublisher;

   @BeforeEach
   public void setup() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      quadrupedTestFactory.setUseNetworking(true);


   }

   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;
      soleTrajectoryPublisher = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testStandingUpAndMovingFoot() throws IOException
   {
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(quadrupedTestFactory.getRobotName());
      soleTrajectoryPublisher = ROS2Tools.createPublisher(stepTeleopManager.getRos2Node(), SoleTrajectoryMessage.class, controllerSubGenerator);

      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      RobotQuadrant quadrantToTest = RobotQuadrant.FRONT_RIGHT;

      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(quadrupedTestFactory.getFullRobotModel());
      referenceFrames.updateFrames();

      FramePoint3D soleBasePosition = new FramePoint3D(referenceFrames.getSoleFrame(quadrantToTest));
      soleBasePosition.changeFrame(worldFrame);

      double epsilon = 0.05;
      runMovingFoot(quadrantToTest, soleBasePosition.getX(), soleBasePosition.getY(), soleBasePosition.getZ() + 0.05, 1.0, epsilon);
      runMovingFoot(quadrantToTest, soleBasePosition.getX() + 0.1, soleBasePosition.getY(), soleBasePosition.getZ() + 0.1, 1.0, epsilon);
      runMovingFoot(quadrantToTest, soleBasePosition.getX() + 0.1, soleBasePosition.getY() + 0.1, soleBasePosition.getZ() + 0.1, 1.0, epsilon);
      runMovingFoot(quadrantToTest, soleBasePosition.getX() - 0.1, soleBasePosition.getY() - 0.1, soleBasePosition.getZ() + 0.1, 1.0, epsilon);
      runMovingFoot(quadrantToTest, soleBasePosition.getX() - 0.1, soleBasePosition.getY() + 0.1, soleBasePosition.getZ() + 0.1, 1.0, epsilon);
      runMovingFoot(quadrantToTest, soleBasePosition.getX() , soleBasePosition.getY(), soleBasePosition.getZ() + 0.05, 1.0, epsilon);
      runMovingFoot(quadrantToTest, soleBasePosition.getX(), soleBasePosition.getY(), soleBasePosition.getZ() - 0.03, 1.0, epsilon);
   }

   @Test
   public void testFootLoadBearingMessage() throws IOException
   {
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      MessageTopicNameGenerator controllerSubGenerator = QuadrupedControllerAPIDefinition.getSubscriberTopicNameGenerator(quadrupedTestFactory.getRobotName());
      soleTrajectoryPublisher = ROS2Tools.createPublisher(stepTeleopManager.getRos2Node(), SoleTrajectoryMessage.class, controllerSubGenerator);

      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      RobotQuadrant quadrantToTest = RobotQuadrant.FRONT_RIGHT;

      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(quadrupedTestFactory.getFullRobotModel());
      referenceFrames.updateFrames();

      FramePoint3D soleBasePosition = new FramePoint3D(referenceFrames.getSoleFrame(quadrantToTest));
      soleBasePosition.changeFrame(worldFrame);

      double epsilon = 0.05;
      runMovingFoot(quadrantToTest, soleBasePosition.getX(), soleBasePosition.getY(), soleBasePosition.getZ() + 0.05, 1.0, epsilon);
      Assertions.assertTrue(variables.getSteppingState().getEnumValue() == QuadrupedSteppingStateEnum.SOLE_WAYPOINT,
                            "Controller did not transition to sole waypoint mode after receiving a sole trajectory message. Is in state: " + variables
                                  .getSteppingState().getEnumValue());

      // check that sending other quadrants does nothing
      stepTeleopManager.requestLoadBearing(quadrantToTest.getAcrossBodyQuadrant());
      stepTeleopManager.requestLoadBearing(quadrantToTest.getDiagonalOppositeQuadrant());
      stepTeleopManager.requestLoadBearing(quadrantToTest.getSameSideQuadrant());

      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 2.0));
      Assertions.assertTrue(variables.getSteppingState().getEnumValue() == QuadrupedSteppingStateEnum.SOLE_WAYPOINT,
                            "Controller transitioned out of sole waypoint mode after receiving a load bearing message for a loaded foot.");

      stepTeleopManager.requestLoadBearing(quadrantToTest);
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 2.0));
      conductor.simulate();
      Assertions.assertTrue(variables.getSteppingState().getEnumValue() == QuadrupedSteppingStateEnum.STAND,
                            "Controller did not transition to stand after receiving a load bearing message.");
   }

   private void runMovingFoot(RobotQuadrant robotQuadrant, double footPositionX, double footPositionY, double footPositionZ, double time, double translationDelta)
   {

      SoleTrajectoryMessage soleTrajectoryCommand = new SoleTrajectoryMessage();
      soleTrajectoryCommand.setRobotQuadrant(robotQuadrant.toByte());
      EuclideanTrajectoryPointMessage trajectoryPointMessage = soleTrajectoryCommand.getPositionTrajectory().getTaskspaceTrajectoryPoints().add();
      trajectoryPointMessage.setTime(time);
      trajectoryPointMessage.getPosition().set(footPositionX, footPositionY, footPositionZ);

      soleTrajectoryPublisher.publish(soleTrajectoryCommand);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 3.0 * time));
      conductor.simulate();

      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(quadrupedTestFactory.getFullRobotModel());
      referenceFrames.updateFrames();

      FramePoint3D solePosition = new FramePoint3D(referenceFrames.getSoleFrame(robotQuadrant));
      solePosition.changeFrame(worldFrame);

      assertEquals("X position is invalid", footPositionX, solePosition.getX(), translationDelta);
      assertEquals("Y position is invalid", footPositionY, solePosition.getY(), translationDelta);
      assertEquals("Z position is invalid", footPositionZ, solePosition.getZ(), translationDelta);
   }

}
