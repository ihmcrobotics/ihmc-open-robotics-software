package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Created by agrabertilton on 2/25/15.
 */
public abstract class HumanoidSwingTrajectoryTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;
   private SCS2AvatarTestingSimulation simulationTestHelper;

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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private class TestController implements Controller
   {
      YoRegistry registry = new YoRegistry("SwingHeightTestController");
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
      public YoRegistry getYoRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return getClass().getSimpleName();
      }

      public double getMaxFootHeight()
      {
         return maxFootHeight.getDoubleValue();
      }
   }

   @Test
   public void testMultipleHeightFootsteps()
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
         simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                               flatGroundEnvironment,
                                                                                               simulationTestingParameters);
         simulationTestHelper.start();
         FullHumanoidRobotModel estimatorRobotModel = simulationTestHelper.getControllerFullRobotModel();
         TestController testController = new TestController(estimatorRobotModel);
         simulationTestHelper.getRobot().addController(testController);

         success = simulationTestHelper.simulateNow(2.0); // 2.0);

         FootstepDataListMessage footstepDataList = createBasicFootstepFromDefaultForSwingHeightTest(currentHeight);
         simulationTestHelper.publishToController(footstepDataList);
         success = success && simulationTestHelper.simulateNow(4.0);
         maxHeights[i] = testController.getMaxFootHeight();
         assertTrue(success);

         if (i != heights.length - 1)
            simulationTestHelper.finishTest();
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

   @Test
   public void testReallyHighFootstep()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setRunMultiThreaded(false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success;
      double currentHeight = 10.0;
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            flatGroundEnvironment,
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      success = simulationTestHelper.simulateNow(2.0); // 2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSwingHeightTest(currentHeight);
      simulationTestHelper.publishToController(footstepDataList);
      success = success && simulationTestHelper.simulateNow(8.0);
      assertTrue(success);

      Point3D center = new Point3D(1.2, 0.0, .75);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testNegativeSwingHeight()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setRunMultiThreaded(false);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success;
      double currentHeight = -0.1;
      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                            flatGroundEnvironment,
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      success = simulationTestHelper.simulateNow(2.0); // 2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSwingHeightTest(currentHeight);
      simulationTestHelper.publishToController(footstepDataList);
      success = success && simulationTestHelper.simulateNow(6.0);
      assertTrue(success);

      Point3D center = new Point3D(1.2, 0.0, .75);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FootstepDataListMessage createBasicFootstepFromDefaultForSwingHeightTest(double swingHeight)
   {
      FootstepDataListMessage desiredFootsteps = HumanoidMessageTools.createFootstepDataListMessage(0.0, 0.0);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(0.4, -0.125, 0.0), new Quaternion(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.getFootstepDataList().add().set(footstep);

      return desiredFootsteps;
   }

   private FootstepDataListMessage createFootstepsForSwingHeightTest(double swingHeight)
   {
      FootstepDataListMessage desiredFootsteps = HumanoidMessageTools.createFootstepDataListMessage(0.0, 0.0);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(0.6, -0.125, 0.0), new Quaternion(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.getFootstepDataList().add().set(footstep);

      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(1.2, 0.125, 0.0), new Quaternion(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.getFootstepDataList().add().set(footstep);

      footstep = HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(1.2, -0.125, 0.0), new Quaternion(0, 0, 0, 1));
      footstep.setTrajectoryType(TrajectoryType.OBSTACLE_CLEARANCE.toByte());
      footstep.setSwingHeight(swingHeight);
      desiredFootsteps.getFootstepDataList().add().set(footstep);
      return desiredFootsteps;
   }

   @Test
   public void testSelfCollisionAvoidance()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGroundEnvironment = new FlatGroundEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             flatGroundEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(new OffsetAndYawRobotInitialSetup(0.0, 1.0, 0.0, Math.toRadians(170.0)));
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      referenceFrames.updateFrames();
      SideDependentList<ArrayList<Point2D>> footContactPoints = new SideDependentList<>(robotModel.getContactPointParameters().getFootContactPoints());
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D footPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footContactPoints.get(robotSide)));
         ConvexPolygon2D shrunkFootPolygon = new ConvexPolygon2D();
         scaler.scaleConvexPolygon(footPolygon, 0.025, shrunkFootPolygon);
         footPolygons.put(robotSide, shrunkFootPolygon);
      }
      CollisionDetector collisionDetector = new CollisionDetector(referenceFrames, footPolygons);
      simulationTestHelper.addRobotControllerOnControllerThread(collisionDetector);

      ThreadTools.sleep(1000);
      Assert.assertTrue(simulationTestHelper.simulateNow(0.5));

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
         message.getFootstepDataList().add().set(footstepData);
         swingSide = swingSide.getOppositeSide();
      }

      simulationTestHelper.publishToController(message);
      double simulationTime = initialTransfer + steps * stepTime + 0.5;
      while (simulationTestHelper.getSimulationTime() < simulationTime)
      {
         Assert.assertTrue(simulationTestHelper.simulateNow(0.5));
         Assert.assertFalse(collisionDetector.didFeetCollide());
      }
   }

   private class CollisionDetector extends SimpleRobotController
   {
      private final CommonHumanoidReferenceFrames referenceFrames;
      private final SideDependentList<ConvexPolygon2D> footPolygonsInSole;
      private final SideDependentList<ConvexPolygon2D> footPolygonsInWorld;
      private final FrameConvexPolygon2D framePolygon = new FrameConvexPolygon2D();

      private boolean collision = false;

      public CollisionDetector(CommonHumanoidReferenceFrames referenceFrames, SideDependentList<ConvexPolygon2D> footPolygons)
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
            footPolygonsInWorld.get(robotSide).set(framePolygon);
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
