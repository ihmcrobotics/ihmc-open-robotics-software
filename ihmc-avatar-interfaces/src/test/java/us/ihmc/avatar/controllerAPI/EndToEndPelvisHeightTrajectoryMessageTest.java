package us.ihmc.avatar.controllerAPI;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.CenterOfMassHeightControlState;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.commonWalkingControlModules.heightPlanning.HeightOffsetHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryExecutionStatus;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class EndToEndPelvisHeightTrajectoryMessageTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private static final boolean DEBUG = false;

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @Test
   public void testSingleWaypoint() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      List<TaskspaceTrajectoryStatusMessage> statusMessages = new ArrayList<>();
      simulationTestHelper.createSubscriberFromController(TaskspaceTrajectoryStatusMessage.class, statusMessages::add);
      double controllerDT = getRobotModel().getControllerDT();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      FramePoint3D desiredRandomPelvisPosition = getRandomPelvisPosition(random, pelvis);
      Point3D desiredPosition = new Point3D(desiredRandomPelvisPosition);

      if (DEBUG)
      {
         System.out.println(desiredPosition);
      }

      desiredRandomPelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPosition.set(desiredRandomPelvisPosition);
      if (DEBUG)
      {
         System.out.println(desiredPosition);
      }

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime,
                                                                                                                             desiredPosition.getZ());
      pelvisHeightTrajectoryMessage.setSequenceId(random.nextLong());

      simulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);

      success = simulationTestHelper.simulateNow(3.0 + trajectoryTime);
      assertTrue(success);

      // Hard to figure out how to verify the desired there
      //      trajOutput = scs.getVariable("pelvisHeightOffsetSubTrajectoryCubicPolynomialTrajectoryGenerator", "pelvisHeightOffsetSubTrajectoryCurrentValue").getValueAsDouble();
      //      assertEquals(desiredPosition.getZ(), trajOutput, epsilon);
      // Ending up doing a rough check on the actual height
      double pelvisHeight = simulationTestHelper.findVariable("PelvisLinearStateUpdater", "estimatedRootJointPositionZ").getValueAsDouble();
      assertEquals(desiredPosition.getZ(), pelvisHeight, 0.01);

      assertEquals(2, statusMessages.size());
      Point3D expectedPosition = new Point3D(Double.NaN, Double.NaN, pelvisHeight);
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(pelvisHeightTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.STARTED,
                                                        0.0,
                                                        "pelvisHeight",
                                                        statusMessages.remove(0),
                                                        controllerDT);
      EndToEndTestTools.assertTaskspaceTrajectoryStatus(pelvisHeightTrajectoryMessage.getSequenceId(),
                                                        TrajectoryExecutionStatus.COMPLETED,
                                                        trajectoryTime,
                                                        expectedPosition,
                                                        null,
                                                        "pelvisHeight",
                                                        statusMessages.remove(0),
                                                        1.0e-3,
                                                        controllerDT);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   public void testSingleWaypointWithControlFrame() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);
      assertTrue(success);

      RigidBodyBasics pelvis = simulationTestHelper.getControllerFullRobotModel().getPelvis();
      MovingReferenceFrame pelvisBodyFrame = pelvis.getBodyFixedFrame();
      FramePoint3D expectedPosition = new FramePoint3D(pelvisBodyFrame);
      expectedPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double trajectoryTime = 0.1;
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime,
                                                                                                                             expectedPosition.getZ());

      pelvisHeightTrajectoryMessage.getEuclideanTrajectory().setUseCustomControlFrame(true);

      simulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);
      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      FramePoint3D actualPosition = new FramePoint3D(pelvisBodyFrame);
      actualPosition.changeFrame(ReferenceFrame.getWorldFrame());

      EuclidCoreTestTools.assertEquals(expectedPosition, actualPosition, 2.0e-3);
   }

   protected FramePoint3D getRandomPelvisPosition(Random random, RigidBodyBasics pelvis)
   {
      FramePoint3D desiredRandomPelvisPosition = new FramePoint3D(pelvis.getParentJoint().getFrameAfterJoint());
      desiredRandomPelvisPosition.set(RandomGeometry.nextPoint3D(random, 0.10, 0.20, 0.05));
      desiredRandomPelvisPosition.setZ(desiredRandomPelvisPosition.getZ());
      return desiredRandomPelvisPosition;
   }

   public void testSingleWaypointInUserMode() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      FramePoint3D desiredRandomPelvisPosition = getRandomPelvisPosition(random, pelvis);
      Point3D desiredPosition = new Point3D(desiredRandomPelvisPosition);

      System.out.println(desiredPosition);

      desiredRandomPelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPosition.set(desiredRandomPelvisPosition);
      System.out.println(desiredPosition);

      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime,
                                                                                                                             desiredPosition.getZ());

      pelvisHeightTrajectoryMessage.setEnableUserPelvisControl(true);
      simulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);

      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      double pelvisHeight = fullRobotModel.getPelvis().getParentJoint().getFrameAfterJoint().getTransformToWorldFrame().getTranslationZ();
      assertEquals(desiredPosition.getZ(), pelvisHeight, 0.01);

      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   public void testSingleWaypointThenManualChange() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      assertTrue(simulationTestHelper.simulateNow(1.0));

      String namespace = HeightOffsetHandler.class.getSimpleName();
      YoDouble offsetHeight = (YoDouble) simulationTestHelper.findVariable(namespace, "offsetHeightAboveGround");

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();
      MovingReferenceFrame pelvisFrame = pelvis.getBodyFixedFrame();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvisFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      double initialPelvisHeight = pelvisPosition.getZ();

      Random random = new Random(4929L);
      for (int i = 0; i < 5; i++)
      {
         double offset1 = 0.06 * 2.0 * (random.nextDouble() - 0.5);
         double offset2 = 0.06 * 2.0 * (random.nextDouble() - 0.5);

         // Move pelvis using YoVariable
         offsetHeight.set(offset1);
         assertTrue(simulationTestHelper.simulateNow(1.5));
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         assertEquals(initialPelvisHeight + offset1, pelvisPosition.getZ(), 0.01);

         // Move pelvis through message
         double desiredHeight = initialPelvisHeight + offset2;
         PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, desiredHeight);
         simulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);
         assertTrue(simulationTestHelper.simulateNow(1.5));
         pelvisPosition.setToZero(pelvisFrame);
         pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
         assertEquals(desiredHeight, pelvisPosition.getZ(), 0.01);
      }
   }

   /**
    * This test is to reproduce a bug found on Valkyrie where sending a stop all trajectory would cause
    * the robot to increase its height.
    */
   public void testStopAllTrajectory() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.setCamera(new Point3D(0.4, 0.0, 1.2), new Point3D(0.4, 12.0, 1.2));
      ThreadTools.sleep(1000);

      // Apply a push to the robot so we get some tracking error going
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      double forceMagnitude = fullRobotModel.getTotalMass() * 2.0;
      String pushJointName = fullRobotModel.getPelvis().getParentJoint().getName();
      PushRobotControllerSCS2 pushController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                                           simulationTestHelper.getRobot(),
                                                                           pushJointName,
                                                                           new Vector3D(),
                                                                           1.0 / forceMagnitude);
      simulationTestHelper.addYoGraphicDefinition(pushController.getForceVizDefinition());
      pushController.applyForce(new Vector3D(0.0, 0.0, 1.0), forceMagnitude, Double.POSITIVE_INFINITY);

      assertTrue(simulationTestHelper.simulateNow(RigidBodyControlManager.INITIAL_GO_HOME_TIME + 1.0));
      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();
      double initialPelvisHeight = referenceFrames.getPelvisFrame().getTransformToWorldFrame().getTranslationZ();

      // Step the trajectory repeatedly
      StopAllTrajectoryMessage stopMessage = new StopAllTrajectoryMessage();
      for (int i = 0; i < 50; i++)
      {
         simulationTestHelper.publishToController(stopMessage);
         assertTrue(simulationTestHelper.simulateNow(0.1));
      }

      // Would be nicer to check desired values but we currently have so many difference height control schemes that that would not be easy.
      referenceFrames.updateFrames();
      double finalPelvisHeight = referenceFrames.getPelvisFrame().getTransformToWorldFrame().getTranslationZ();
      assertEquals(initialPelvisHeight, finalPelvisHeight, 1.0e-3);
   }

   @Test
   public void testStreaming() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      Random random = new Random(54651);
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      YoRegistry testRegistry = new YoRegistry("testStreaming");

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.getRootRegistry().addChild(testRegistry);

      boolean success = simulationTestHelper.simulateNow(1.0);
      assertTrue(success);

      YoDouble startTime = new YoDouble("startTime", testRegistry);
      YoDouble yoTime = simulationTestHelper.getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox().getYoTime();
      startTime.set(yoTime.getValue());
      YoDouble trajectoryTime = new YoDouble("trajectoryTime", testRegistry);
      trajectoryTime.set(2.0);

      YoDouble initialHeight = new YoDouble("initialHeight", testRegistry);
      YoDouble finalHeight = new YoDouble("finalHeight", testRegistry);
      YoDouble desiredHeight = new YoDouble("desiredHeight", testRegistry);
      YoDouble desiredHeightRate = new YoDouble("desiredHeightRate", testRegistry);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      RigidBodyBasics pelvis = fullRobotModel.getPelvis();

      FramePoint3D pelvisPosition = new FramePoint3D(pelvis.getBodyFixedFrame());
      pelvisPosition.changeFrame(worldFrame);

      initialHeight.set(pelvisPosition.getZ());
      finalHeight.set(pelvisPosition.getZ() + RandomNumbers.nextDouble(random, 0.1));

      simulationTestHelper.addRobotControllerOnControllerThread(new RobotController()
      {
         @Override
         public void initialize()
         {
         }

         private boolean everyOtherTick = false;

         @Override
         public void doControl()
         {
            everyOtherTick = !everyOtherTick;

            if (!everyOtherTick)
               return;

            double timeInTrajectory = yoTime.getValue() - startTime.getValue();
            timeInTrajectory = MathTools.clamp(timeInTrajectory, 0.0, trajectoryTime.getValue());
            double alpha = timeInTrajectory / trajectoryTime.getValue();

            desiredHeight.set(EuclidCoreTools.interpolate(initialHeight.getValue(), finalHeight.getValue(), alpha));
            if (alpha <= 0.0 || alpha >= 1.0)
               desiredHeightRate.set(0.0);
            else
               desiredHeightRate.set((finalHeight.getValue() - initialHeight.getValue()) / trajectoryTime.getValue());
            PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.0,
                                                                                                             desiredHeight.getValue(),
                                                                                                             desiredHeightRate.getValue());
            message.getEuclideanTrajectory().getQueueingProperties().setExecutionMode(ExecutionMode.STREAM.toByte());
            message.getEuclideanTrajectory().getQueueingProperties().setStreamIntegrationDuration(0.01);
            simulationTestHelper.publishToController(message);
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return null;
         }

         @Override
         public String getDescription()
         {
            return RobotController.super.getDescription();
         }

         @Override
         public String getName()
         {
            return RobotController.super.getName();
         }
      });

      success = simulationTestHelper.simulateNow(0.5 * trajectoryTime.getValue());
      assertTrue(success);

      YoDouble controllerHeight = EndToEndTestTools.findYoDouble(CenterOfMassHeightControlState.class.getSimpleName(),
                                                                 "desiredCoMHeightFromTrajectory",
                                                                 simulationTestHelper);
      YoDouble controllerHeightRate = EndToEndTestTools.findYoDouble("pelvisHeightOffsetMultipleWaypointsTrajectoryGenerator",
                                                                     "pelvisHeightOffsetSubTrajectoryCurrentVelocity",
                                                                     simulationTestHelper);

      assertEquals(desiredHeight.getValue(), controllerHeight.getValue(), 5.0e-4);
      assertEquals(desiredHeightRate.getValue(), controllerHeightRate.getValue(), 1.0e-7);

      success = simulationTestHelper.simulateNow(0.5 * trajectoryTime.getValue() + 1.5);
      assertTrue(success);

      assertEquals(desiredHeight.getValue(), controllerHeight.getValue(), 1.0e-7);
      assertEquals(desiredHeightRate.getValue(), controllerHeightRate.getValue(), 1.0e-7);
      // TODO Not performing assertions on the tracking as all the smoothing stuff is ruining it.
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
