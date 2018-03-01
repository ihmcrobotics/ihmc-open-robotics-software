package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Created by agrabertilton on 2/25/15.
 */
public abstract class DRCSwingTrajectoryTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private class TestController implements RobotController
   {
      YoVariableRegistry registry = new YoVariableRegistry("SwingHeightTestController");
      Random random = new Random();
      FullHumanoidRobotModel estimatorModel;
      YoDouble maxFootHeight = new YoDouble("maxFootHeight", registry);
      YoDouble leftFootHeight = new YoDouble("leftFootHeight", registry);
      YoDouble rightFootHeight = new YoDouble("rightFootHeight", registry);
      YoDouble randomNum = new YoDouble("randomNumberInTestController", registry);
      FramePoint3D leftFootOrigin;
      FramePoint3D rightFootOrigin;

      public TestController(FullHumanoidRobotModel estimatorModel)
      {
         this.estimatorModel = estimatorModel;
      }

      @Override
      public void doControl()
      {
         ReferenceFrame leftFootFrame = estimatorModel.getFoot(RobotSide.LEFT).getBodyFixedFrame();
         leftFootOrigin = new FramePoint3D(leftFootFrame);
         leftFootOrigin.changeFrame(ReferenceFrame.getWorldFrame());

         ReferenceFrame rightFootFrame = estimatorModel.getFoot(RobotSide.RIGHT).getBodyFixedFrame();
         rightFootOrigin = new FramePoint3D(rightFootFrame);
         rightFootOrigin.changeFrame(ReferenceFrame.getWorldFrame());

         randomNum.set(random.nextDouble());
         leftFootHeight.set(leftFootOrigin.getZ());
         rightFootHeight.set(rightFootOrigin.getZ());
         maxFootHeight.set(Math.max(maxFootHeight.getDoubleValue(), leftFootHeight.getDoubleValue()));
         maxFootHeight.set(Math.max(maxFootHeight.getDoubleValue(), rightFootHeight.getDoubleValue()));
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return null;
      }

      @Override
      public String getDescription()
      {
         return null;
      }

      public double getMaxFootHeight()
      {
         return maxFootHeight.getDoubleValue();
      }
   }


   public void testMultipleHeightFootsteps() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setRunMultiThreaded(false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double[] heights = {0.1, 0.2, 0.3};
      double[] maxHeights = new double[heights.length];
      boolean success;
      for (int i = 0; i < heights.length; i++)
      {
         double currentHeight = heights[i];
         FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
         drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
         drcSimulationTestHelper.setTestEnvironment(flatGroundEnvironment);
         drcSimulationTestHelper.createSimulation("DRCWalkingOverSmallPlatformTest");
         FullHumanoidRobotModel estimatorRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
         TestController testController = new TestController(estimatorRobotModel);
         drcSimulationTestHelper.getRobot().setController(testController, 1);

         SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();


         ThreadTools.sleep(1000);
         success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);

         FootstepDataListMessage footstepDataList = createBasicFootstepFromDefaultForSwingHeightTest(currentHeight);
         drcSimulationTestHelper.send(footstepDataList);
         success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
         maxHeights[i] = testController.getMaxFootHeight();
         assertTrue(success);

         if (i != heights.length - 1)
            drcSimulationTestHelper.destroySimulation();
      }

      for (int i = 0; i < heights.length - 1; i++)
      {
         if (heights[i] > heights[i + 1])
         {
            assertTrue(maxHeights[i] > maxHeights[i + 1]);
         }
         else
         {
            assertTrue(maxHeights[i] < maxHeights[i + 1]);
         }
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testReallyHighFootstep() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setRunMultiThreaded(false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success;
      double currentHeight = 10.0;
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGroundEnvironment);
      drcSimulationTestHelper.createSimulation("DRCWalkingOverSmallPlatformTest");

      ThreadTools.sleep(1000);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSwingHeightTest(currentHeight);
      drcSimulationTestHelper.send(footstepDataList);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(8.0);
      assertTrue(success);

      Point3D center = new Point3D(1.2, 0.0, .75);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testNegativeSwingHeight() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setRunMultiThreaded(false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success;
      double currentHeight = -0.1;
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGroundEnvironment);
      drcSimulationTestHelper.createSimulation("DRCWalkingOverSmallPlatformTest");

      ThreadTools.sleep(1000);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSwingHeightTest(currentHeight);
      drcSimulationTestHelper.send(footstepDataList);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);
      assertTrue(success);

      Point3D center = new Point3D(1.2, 0.0, .75);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FootstepDataListMessage createBasicFootstepFromDefaultForSwingHeightTest(double swingHeight)
   {
      FootstepDataListMessage desiredFootsteps = HumanoidMessageTools.createFootstepDataListMessage(0.0, 0.0);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(0.4, -0.125, 0.0), new Quaternion(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.footstepDataList.add(footstep);

      return desiredFootsteps;
   }

   private FootstepDataListMessage createFootstepsForSwingHeightTest(double swingHeight)
   {
      FootstepDataListMessage desiredFootsteps = HumanoidMessageTools.createFootstepDataListMessage(0.0, 0.0);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(0.6, -0.125, 0.0), new Quaternion(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.footstepDataList.add(footstep);

      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(1.2, 0.125, 0.0), new Quaternion(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.footstepDataList.add(footstep);

      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(1.2, -0.125, 0.0), new Quaternion(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.footstepDataList.add(footstep);
      return desiredFootsteps;
   }

   public void testSelfCollisionAvoidance() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(0.0, 1.0, 0.0, Math.toRadians(170.0)));
      drcSimulationTestHelper.setTestEnvironment(flatGroundEnvironment);
      drcSimulationTestHelper.createSimulation("TestSelfCollisionAvoidance");

      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(drcSimulationTestHelper.getControllerFullRobotModel());
      referenceFrames.updateFrames();
      SideDependentList<ArrayList<Point2D>> footContactPoints = robotModel.getContactPointParameters().getFootContactPoints();
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D footPolygon = new ConvexPolygon2D(footContactPoints.get(robotSide));
         ConvexPolygon2D shrunkFootPolygon = new ConvexPolygon2D();
         scaler.scaleConvexPolygon(footPolygon, 0.025, shrunkFootPolygon);
         footPolygons.put(robotSide, shrunkFootPolygon);
      }
      CollisionDetector collisionDetector = new CollisionDetector(referenceFrames, footPolygons);
      drcSimulationTestHelper.addRobotControllerOnControllerThread(collisionDetector);

      ThreadTools.sleep(1000);
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      double referenceLength = walkingControllerParameters.nominalHeightAboveAnkle();
      double swingTime = 1.0;
      double stepTime = walkingControllerParameters.getDefaultTransferTime() + swingTime;
      double initialTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      Random random = new Random(391931L);
      RobotSide swingSide = RobotSide.LEFT;
      int steps = 20;

      Point3D lastPosition = new Point3D();
      MovingReferenceFrame stanceFrame = referenceFrames.getSoleZUpFrame(swingSide.getOppositeSide());
      FramePoint3D initialStance = new FramePoint3D(stanceFrame);
      initialStance.changeFrame(ReferenceFrame.getWorldFrame());
      lastPosition.set(initialStance);
      FrameQuaternion stepOrientation = new FrameQuaternion(stanceFrame);
      stepOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepDataListMessage message = new FootstepDataListMessage();
      for (int i = 0; i < steps; i++)
      {
         FramePoint3D stepLocation = new FramePoint3D(ReferenceFrame.getWorldFrame());
         double angle = Math.PI / 2.0 * (random.nextDouble() - 0.5);
         if (random.nextBoolean())
         {
            angle += Math.PI;
         }
         double radius = referenceLength / 2.5;
         stepLocation.setX(radius * Math.cos(angle));
         stepLocation.setY(radius * Math.sin(angle));
         stepLocation.add(lastPosition);

         lastPosition.set(stepLocation);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(swingSide, stepLocation, stepOrientation);
         footstepData.setSwingDuration(swingTime);
         message.add(footstepData);
         swingSide = swingSide.getOppositeSide();
      }

      drcSimulationTestHelper.send(message);
      double simulationTime = initialTransfer + steps * stepTime + 0.5;
      while (drcSimulationTestHelper.getYoVariable("t").getValueAsDouble() < simulationTime)
      {
         Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
         Assert.assertFalse(collisionDetector.didFeetCollide());
      }
   }

   private class CollisionDetector extends SimpleRobotController
   {
      private final HumanoidReferenceFrames referenceFrames;
      private final SideDependentList<ConvexPolygon2D> footPolygonsInSole;
      private final SideDependentList<ConvexPolygon2D> footPolygonsInWorld;
      private final FrameConvexPolygon2d framePolygon = new FrameConvexPolygon2d();

      private boolean collision = false;

      public CollisionDetector(HumanoidReferenceFrames referenceFrames, SideDependentList<ConvexPolygon2D> footPolygons)
      {
         this.referenceFrames = referenceFrames;
         this.footPolygonsInSole = footPolygons;
         this.footPolygonsInWorld = new SideDependentList<>(new ConvexPolygon2D(), new ConvexPolygon2D());
      }

      @Override
      public void doControl()
      {
         referenceFrames.updateFrames();
         for (RobotSide robotSide : RobotSide.values)
         {
            MovingReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
            framePolygon.setIncludingFrame(soleFrame, footPolygonsInSole.get(robotSide));
            framePolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
            footPolygonsInWorld.get(robotSide).setAndUpdate(framePolygon.getConvexPolygon2d());
         }

         ConvexPolygon2D leftPolygon = footPolygonsInWorld.get(RobotSide.LEFT);
         ConvexPolygon2D rightPolygon = footPolygonsInWorld.get(RobotSide.RIGHT);
         boolean overlap = false;

         for (int vertexIdx = 0; vertexIdx < rightPolygon.getNumberOfVertices(); vertexIdx++)
         {
            Point2DReadOnly vertex = rightPolygon.getVertex(vertexIdx);
            boolean vertexInsideLeft = leftPolygon.isPointInside(vertex);

            if (vertexInsideLeft)
            {
               overlap = true;
               break;
            }
         }

         if (overlap)
         {
            collision = true;
         }
      }

      public boolean didFeetCollide()
      {
         return collision;
      }
   }

}
