package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.LegTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class EndToEndLegTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static boolean DEBUG = false;

   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   protected double getTimePerWaypoint()
   {
      return 0.5;
   }

   protected boolean usePerfectSensors()
   {
      return false;
   }

   //the z height the foot comes off the ground before starting the trajectory
   protected double getLiftOffHeight()
   {
      return 0.15;
   }

   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564654L);
      double epsilon = 1.0e-10;

      createSimulationTestHelper();
      simulationTestHelper.start();

      List<JointspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(JointspaceTrajectoryStatusMessage.class, statusMessages::add);

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         double trajectoryTime = 0.5;
         RigidBodyBasics pelvis = fullRobotModel.getPelvis();
         RigidBodyBasics foot = fullRobotModel.getFoot(robotSide);
         FramePose3D initialFootPosition = new FramePose3D(foot.getBodyFixedFrame());
         initialFootPosition.changeFrame(ReferenceFrame.getWorldFrame());

         // First need to pick up the foot:
         assertTrue(EndToEndFootTrajectoryMessageTest.pickupFoot(robotSide, foot, getLiftOffHeight(), simulationTestHelper));

         OneDoFJointBasics[] legJoints = MultiBodySystemTools.createOneDoFJointPath(pelvis, foot);
         String[] legJointNames = Stream.of(legJoints).map(JointReadOnly::getName).toArray(String[]::new);
         int numberOfJoints = MultiBodySystemTools.computeDegreesOfFreedom(legJoints);
         double[] desiredJointPositions = EndToEndArmTrajectoryMessageTest.generateRandomJointPositions(random, legJoints);
         for (int i = 0; i < legJoints.length; i++)
         { // Tweaking the knee joint angle
            if (legJoints[i] == fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH))
            {
               double qMin = legJoints[i].getJointLimitLower();
               double qMax = legJoints[i].getJointLimitUpper();
               if (legJoints[i].getQ() > 0.0)
                  qMin = legJoints[i].getQ();
               else
                  qMax = legJoints[i].getQ();
               desiredJointPositions[i] = EuclidCoreRandomTools.nextDouble(random, qMin, qMax);
            }
         }
         double[] desiredJointVelocities = new double[numberOfJoints];
         long sequenceID = random.nextLong();

         EndToEndArmTrajectoryMessageTest.generateRandomJointPositions(random, legJoints);

         LegTrajectoryMessage legTrajectoryMessage = HumanoidMessageTools.createLegTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);
         legTrajectoryMessage.setSequenceId(sequenceID);

         if (DEBUG)
         {
            for (int i = 0; i < numberOfJoints; i++)
            {
               OneDoFJointBasics legJoint = legJoints[i];
               System.out.println(legJoint.getName() + ": q = " + legJoint.getQ());
            }
         }

         simulationTestHelper.publishToController(legTrajectoryMessage);

         success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
         assertTrue(success);

         EndToEndTestTools.assertOneDoFJointsFeebackControllerDesireds(legJointNames,
                                                                       desiredJointPositions,
                                                                       desiredJointVelocities,
                                                                       epsilon,
                                                                       simulationTestHelper);

         assertEquals(2, statusMessages.size());
         JointspaceTrajectoryStatusMessage startedStatus = statusMessages.remove(0);
         JointspaceTrajectoryStatusMessage completedStatus = statusMessages.remove(0);
         EndToEndTestTools.assertJointspaceTrajectoryStatus(sequenceID,
                                                            TrajectoryExecutionStatus.STARTED,
                                                            0.0,
                                                            legJointNames,
                                                            startedStatus,
                                                            getRobotModel().getControllerDT());
         EndToEndTestTools.assertJointspaceTrajectoryStatus(sequenceID,
                                                            TrajectoryExecutionStatus.COMPLETED,
                                                            trajectoryTime,
                                                            desiredJointPositions,
                                                            legJointNames,
                                                            completedStatus,
                                                            1.0e-12,
                                                            getRobotModel().getControllerDT());

         // Without forgetting to put the foot back on the ground
         prepFootForLoadBearing(robotSide, foot, initialFootPosition, simulationTestHelper);
         assertTrue(EndToEndFootTrajectoryMessageTest.putFootOnGround(robotSide, foot, initialFootPosition, simulationTestHelper));
      }

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   static boolean prepFootForLoadBearing(RobotSide robotSide, RigidBodyBasics foot, FramePose3D desiredPose, SCS2AvatarTestingSimulation simulationTestHelper)
         throws SimulationExceededMaximumTimeException
   {
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      double trajectoryTime = 1.0;

      desiredPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      desiredPose.get(desiredPosition, desiredOrientation);
      desiredPosition.addZ(0.2);

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide,
                                                                                                     trajectoryTime,
                                                                                                     desiredPosition,
                                                                                                     desiredOrientation);
      simulationTestHelper.publishToController(footTrajectoryMessage);

      return simulationTestHelper.simulateNow(trajectoryTime);
   }

   public void createSimulationTestHelper()
   {
      SCS2AvatarTestingSimulationFactory testSimulationFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                       simulationTestingParameters);
      testSimulationFactory.setUsePerfectSensors(usePerfectSensors());
      simulationTestHelper = testSimulationFactory.createAvatarTestingSimulation();
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
