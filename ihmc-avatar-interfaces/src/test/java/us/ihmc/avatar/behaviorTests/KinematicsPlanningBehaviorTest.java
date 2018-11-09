package us.ihmc.avatar.behaviorTests;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule.KinematicsPlanningToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.humanoidBehaviors.behaviors.primitives.KinematicsPlanningBehavior;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class KinematicsPlanningBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private boolean isKinematicsPlanningToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsPlanningToolboxModule kinematicsPlanningToolboxModule;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (true)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }

      if (kinematicsPlanningToolboxModule != null)
      {
         kinematicsPlanningToolboxModule.destroy();
         kinematicsPlanningToolboxModule = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Before
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface envrionment = new FlatGroundEnvironment();

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupKinematicsPlanningToolboxModule();
   }

   @ContinuousIntegrationTest(estimatedDuration = 46.9)
   @Test(timeout = 230000)
   public void testReachToAPoint() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(2.5);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      FullHumanoidRobotModel sdfFullRobotModel = drcBehaviorTestHelper.getSDFFullRobotModel();
      KinematicsPlanningBehavior behavior = new KinematicsPlanningBehavior(drcBehaviorTestHelper.getRobotName(), drcBehaviorTestHelper.getRos2Node(),
                                                                           getRobotModel(), sdfFullRobotModel);

      double trajectoryTime = 5.0;
      int numberOfKeyFrames = 3; // TODO : check the number of key frames. Should be tested with 1.
      RobotSide robotSide = RobotSide.LEFT;

      List<Pose3DReadOnly> desiredPoses = new ArrayList<>();
      Pose3D desiredPose = new Pose3D();
      desiredPose.setPosition(0.5, 0.3, 1.0);
      desiredPose.appendYawRotation(-0.5 * Math.PI);
      desiredPose.appendPitchRotation(0.5 * Math.PI);
      desiredPose.appendYawRotation(0.2 * Math.PI);

      Pose3D initialPose = new Pose3D(sdfFullRobotModel.getHand(robotSide).getBodyFixedFrame().getTransformToWorldFrame());
      for (int i = 0; i < numberOfKeyFrames; i++)
      {
         Pose3D pose = new Pose3D(initialPose);
         double alpha = (i + 1) / (double) (numberOfKeyFrames);
         pose.interpolate(desiredPose, alpha);
         desiredPoses.add(pose);
         drcBehaviorTestHelper.getSimulationConstructionSet().addStaticLinkGraphics(createEndEffectorKeyFrameVisualization(pose));
      }

      behavior.setKeyFrameTimes(trajectoryTime, numberOfKeyFrames);
      behavior.setEndEffectorKeyFrames(robotSide, desiredPoses);

      drcBehaviorTestHelper.updateRobotModel();

      drcBehaviorTestHelper.dispatchBehavior(behavior);

      while (!behavior.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration = 46.9)
   @Test(timeout = 230000)
   public void testRaiseUpHand() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(2.5);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      KinematicsPlanningBehavior behavior = new KinematicsPlanningBehavior(drcBehaviorTestHelper.getRobotName(), drcBehaviorTestHelper.getRos2Node(),
                                                                           getRobotModel(), drcBehaviorTestHelper.getSDFFullRobotModel());

      double trajectoryTime = 5.0;
      int numberOfKeyFrames = 10;
      RobotSide robotSide = RobotSide.LEFT;

      RigidBody endEffector = drcBehaviorTestHelper.getSDFFullRobotModel().getHand(robotSide);

      FramePose3D initialPose = new FramePose3D(endEffector.getBodyFixedFrame());
      FramePose3D desiredPose = new FramePose3D(endEffector.getBodyFixedFrame(), new Point3D(0.1, 0.1, 0.6), new AxisAngle(1.0, 0.0, 0.0, 0.5 * Math.PI));
      initialPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredPose.changeFrame(ReferenceFrame.getWorldFrame());

      TDoubleArrayList keyFrameTimes = new TDoubleArrayList();
      List<Pose3DReadOnly> keyFramePoses = new ArrayList<Pose3DReadOnly>();
      List<Point3DReadOnly> desiredCOMPoints = new ArrayList<Point3DReadOnly>();

      for (int i = 0; i < numberOfKeyFrames; i++)
      {
         double alpha = (i + 1) / (double) (numberOfKeyFrames);
         keyFrameTimes.add(alpha * trajectoryTime);
         Pose3D pose = new Pose3D(initialPose);
         pose.interpolate(desiredPose, alpha);
         keyFramePoses.add(pose);
         desiredCOMPoints.add(new Point3D());
         drcBehaviorTestHelper.getSimulationConstructionSet().addStaticLinkGraphics(createEndEffectorKeyFrameVisualization(pose));
      }

      behavior.setKeyFrameTimes(trajectoryTime, numberOfKeyFrames);
      behavior.setEndEffectorKeyFrames(robotSide, desiredPose);

      drcBehaviorTestHelper.updateRobotModel();

      drcBehaviorTestHelper.dispatchBehavior(behavior);

      while (!behavior.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupKinematicsPlanningToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsPlanningToolboxModule = new KinematicsPlanningToolboxModule(robotModel, isKinematicsPlanningToolboxVisualizerEnabled,
                                                                            PubSubImplementation.INTRAPROCESS);
   }

   private static Graphics3DObject createEndEffectorKeyFrameVisualization(Pose3D pose)
   {
      Graphics3DObject object = new Graphics3DObject();
      object.transform(new RigidBodyTransform(pose.getOrientation(), pose.getPosition()));
      object.addSphere(0.01);

      return object;
   }
}
