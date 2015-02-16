package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.PacketRouter;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWallWorldEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public abstract class DRCDetectCollisionUsingWristSensorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final DRCWallWorldEnvironment testEnvironment = new DRCWallWorldEnvironment(-0.5, 2.5);
   private final PacketCommunicator controllerObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.CONTROLLER.ordinal(), "DRCWallCollisionTestControllerLocalCommunicator");
   private final PacketCommunicator npObjectCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.NETWORK_PROCESSOR.ordinal(), "DRCWallCollisionTestNPLocalCommunicator");

   private Robot robotToTest;
   private final RobotSide robotSideToTest = RobotSide.LEFT;
   private FullRobotModel fullRobotModel;

   private BehaviorCommunicationBridge communicationBridge;

   private WristForceSensorFilteredUpdatable wristUpdatable;
   private UpdateForceSensorFromRobotController updateForceSensorController;
   private GroundContactPoint wristGCPoint;
   private HandImpactDetector handImpactDetector;

   private DoubleYoVariable yoTime;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, controllerObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, false, getRobotModel());

      robotToTest = drcSimulationTestHelper.getRobot();
      fullRobotModel = getRobotModel().createFullRobotModel();
      
      KryoLocalPacketCommunicator behaviorCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
            PacketDestination.BEHAVIOR_MODULE.ordinal(), "behvaiorCommunicator");
      
      PacketRouter networkProcessor = new PacketRouter();
      networkProcessor.attachPacketCommunicator(npObjectCommunicator);
      networkProcessor.attachPacketCommunicator(controllerObjectCommunicator);
      networkProcessor.attachPacketCommunicator(behaviorCommunicator);

      communicationBridge = new BehaviorCommunicationBridge(behaviorCommunicator, robotToTest.getRobotsYoVariableRegistry());

      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));
      RobotDataReceiver robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      controllerObjectCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
      wristUpdatable = setupWristForceSensorUpdatable(forceSensorDataHolder);

      updateForceSensorController = setupUpdateForceSensorController(robotDataReceiver, wristUpdatable);
      robotToTest.setController(updateForceSensorController);

      wristGCPoint = drcSimulationTestHelper.getRobot().getHandGroundContactPoints(robotSideToTest).get(0);

      handImpactDetector = new HandImpactDetector(robotToTest, wristGCPoint);
      robotToTest.setController(handImpactDetector);

      yoTime = robotToTest.getYoTime();

      setupCameraForHandstepsOnWalls();
   }

   private WristForceSensorFilteredUpdatable setupWristForceSensorUpdatable(ForceSensorDataHolder forceSensorDataHolder)
   {
      DRCRobotSensorInformation sensorInfo = getRobotModel().getSensorInformation();
      double DT = drcSimulationTestHelper.getSimulationConstructionSet().getDT();

      WristForceSensorFilteredUpdatable ret = new WristForceSensorFilteredUpdatable(robotSideToTest, fullRobotModel, sensorInfo, forceSensorDataHolder, DT,
            controllerObjectCommunicator, robotToTest.getRobotsYoVariableRegistry());

      return ret;
   }

   private UpdateForceSensorFromRobotController setupUpdateForceSensorController(RobotDataReceiver robotDataReceiver,
         WristForceSensorFilteredUpdatable wristUpdatable)
   {
      ArrayList<WrenchCalculatorInterface> forceSensors = new ArrayList<WrenchCalculatorInterface>();

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      robotToTest.getForceSensors(forceSensors);

      UpdateForceSensorFromRobotController ret = new UpdateForceSensorFromRobotController(robotDataReceiver, wristUpdatable, forceSensors);

      return ret;
   }

   //   @Test(timeout=300000)
   public void testConstrainedHandPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double timeToStartMotionConstraint = 1.3 * 1.0;
      double arrestStiffness_NperM = 10000.0;
      GCPointMotionConstrainer handGCArrestor = new GCPointMotionConstrainer(robotToTest, wristGCPoint, timeToStartMotionConstraint, arrestStiffness_NperM);

      robotToTest.setController(handGCArrestor);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      wristUpdatable.setStopMotionIfCollision(true);

      double trajectoryTime = 2.0;
      double handDeltaX = 0.4;
      double handDeltaY = 0.0;
      double handDeltaZ = 0.0;
      double handDeltaOmega = 0.0;

      success = success && sendHandPoseBehaviorAndSimulate(trajectoryTime, handDeltaX, handDeltaY, handDeltaZ, handDeltaOmega);

      double actualHandGcCollisionTime = handImpactDetector.getHandImpactTime();
      double wristForceSensorCollisionTime = wristUpdatable.getHandImpactTime();

      assertEquals("Expected time of detected impact = " + actualHandGcCollisionTime + ".  Actual detected impact time = " + wristForceSensorCollisionTime,
            actualHandGcCollisionTime, wristForceSensorCollisionTime, 0.1);

      BambooTools.reportTestFinishedMessage();
   }

	@AverageDuration(duration = 15.9)
	@Test(timeout = 47679)
   public void testImpactDetectionUsingHandPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      wristUpdatable.setStopMotionIfCollision(true);

      double trajectoryTime = 2.0;
      double handDeltaX = 0.4;
      double handDeltaY = 0.0;
      double handDeltaZ = 0.0;
      double handDeltaOmega = 0.0;

      success = success && sendHandPoseBehaviorAndSimulate(trajectoryTime, handDeltaX, handDeltaY, handDeltaZ, handDeltaOmega);

      double actualHandGcCollisionTime = handImpactDetector.getHandImpactTime();
      double wristForceSensorCollisionTime = wristUpdatable.getHandImpactTime();

      assertEquals("Expected time of detected impact = " + actualHandGcCollisionTime + ".  Actual detected impact time = " + wristForceSensorCollisionTime,
            actualHandGcCollisionTime, wristForceSensorCollisionTime, 0.1);

      BambooTools.reportTestFinishedMessage();
   }

   //FIXME: Running more than one test causes the following error: java.lang.RuntimeException: java.net.BindException: Address already in use: bind
   //      @Test(timeout=300000)
   public void testHandMassCompensationBySlowlyRotatingHands() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      wristUpdatable.setStopMotionIfCollision(false);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double trajectoryTime = 2.0;
      double handDeltaX = 0.4;
      double handDeltaY = 0.0;
      double handDeltaZ = 0.0;
      double handDeltaOmega = 0.0;

      success = success && sendHandPoseBehaviorAndSimulate(trajectoryTime, handDeltaX, handDeltaY, handDeltaZ, handDeltaOmega);

      double maxWristForceMassCompensated = updateForceSensorController.getMaxWristForceMassCompensated();
      double reasonableMaxValue = 0.5 * Math.abs(9.81 * wristUpdatable.getHandMass());

      System.out.println("MaxWristForceMassCompensated = " + maxWristForceMassCompensated + ".  Should be less than: " + reasonableMaxValue);

      Boolean maxWristForceMassCompensatedIsReasonablySmall = Math.abs(maxWristForceMassCompensated) < reasonableMaxValue;
      assertTrue(maxWristForceMassCompensatedIsReasonablySmall);

      BambooTools.reportTestFinishedMessage();
   }

   private final Quat4d handOrientation = new Quat4d();

   private boolean sendHandPoseBehaviorAndSimulate(final double swingTrajectoryTime, double deltaX, double deltaY, double deltaZ, double deltaOmega)
         throws SimulationExceededMaximumTimeException
   {
      final HandPoseBehavior handPoseBehavior = new HandPoseBehavior(communicationBridge, yoTime);

      final FramePose handPose = new FramePose();
      handPose.setToZero(fullRobotModel.getHandControlFrame(robotSideToTest));

      handPose.changeFrame(ReferenceFrame.getWorldFrame());

      handPose.setX(handPose.getX() + deltaX);
      handPose.setY(handPose.getY() + deltaY);
      handPose.setZ(handPose.getZ() + deltaZ);

      handPose.getOrientation(handOrientation);
      handOrientation.setW(handOrientation.w + deltaOmega);
      handPose.setOrientation(handOrientation);

      final RigidBodyTransform handPoseTransform = new RigidBodyTransform();
      handPose.getPose(handPoseTransform);

      handPoseBehavior.setInput(Frame.WORLD, handPoseTransform, robotSideToTest, swingTrajectoryTime);

      final double simulationRunTime = swingTrajectoryTime + 1.0;

      Thread handPoseBehaviorThread = new Thread()
      {
         public void run()
         {
            {
               double startTime = Double.NaN;
               boolean simStillRunning = true;
               boolean initalized = false;

               while (simStillRunning)
               {
                  if (!initalized)
                  {
                     startTime = yoTime.getDoubleValue();
                     initalized = true;
                  }

                  double timeSpentSimulating = yoTime.getDoubleValue() - startTime;
                  simStillRunning = timeSpentSimulating < simulationRunTime;

                  handPoseBehavior.doControl();
               }
            }
         }
      };

      handPoseBehaviorThread.start();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationRunTime);

      return success;
   }

   private class UpdateForceSensorFromRobotController implements RobotController
   {
      private final YoVariableRegistry registry;
      private final RobotDataReceiver robotDataReceiver;
      private final WristForceSensorFilteredUpdatable wristUpdatable;
      private final DoubleYoVariable maxWristForceMassCompensated;

      public UpdateForceSensorFromRobotController(RobotDataReceiver robotDataReceiver, WristForceSensorFilteredUpdatable wristUpdatable,
            ArrayList<WrenchCalculatorInterface> forceSensors)
      {
         registry = new YoVariableRegistry("ForceSensorController");
         this.robotDataReceiver = robotDataReceiver;
         this.wristUpdatable = wristUpdatable;
         this.maxWristForceMassCompensated = new DoubleYoVariable("maxWristForceMassCompensated", registry);
      }

      public double getMaxWristForceMassCompensated()
      {
         return maxWristForceMassCompensated.getDoubleValue();
      }

      public void initialize()
      {
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public String getName()
      {
         return null;
      }

      public String getDescription()
      {
         return null;
      }

      public void doControl()
      {
         robotDataReceiver.updateRobotModel();
         wristUpdatable.update(yoTime.getDoubleValue());

         double wristForceMassCompensated = wristUpdatable.getWristForceMassCompensatedInWorld().length();
         if (wristForceMassCompensated > maxWristForceMassCompensated.getDoubleValue() && yoTime.getDoubleValue() > 1.0)
         {
            maxWristForceMassCompensated.set(wristForceMassCompensated);
         }

      }
   }

   private class HandImpactDetector implements RobotController
   {
      private final YoVariableRegistry registry;
      private final Robot robot;
      private final GroundContactPoint handGCPoint;
      private final DoubleYoVariable handGCPointCollisionTime;

      public HandImpactDetector(Robot robot, GroundContactPoint handGCPoint)
      {
         registry = new YoVariableRegistry(handGCPoint.getName() + "ImpactDetector");
         this.robot = robot;
         this.handGCPoint = handGCPoint;
         handGCPointCollisionTime = new DoubleYoVariable(handGCPoint.getName() + "CollisionTime", registry);
      }

      public double getHandImpactTime()
      {
         return handGCPointCollisionTime.getDoubleValue();
      }

      boolean handGCHasCollided = false;

      public void doControl()
      {
         if (handGCPoint.isInContact() && !handGCHasCollided)
         {
            handGCPointCollisionTime.set(robot.getTime());
            handGCHasCollided = true;
         }
      }

      public void initialize()
      {
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public String getName()
      {
         return registry.getName();
      }

      public String getDescription()
      {
         return registry.getName();
      }
   }

   private class GCPointMotionConstrainer implements RobotController
   {
      private final YoVariableRegistry registry;
      private final Robot robot;
      private final GroundContactPoint handGCPoint;

      private final double elapsedUnarrestedTime;
      private final double stiffness_NperM;

      private final YoFrameVector allowableDirectionOfMotion;

      private final YoFramePoint desiredPosition;
      private final YoFramePoint initialPosition;
      private final YoFramePoint currentPosition;

      private final YoFrameVector currentPositionFromInitial;
      private final YoFrameVector desiredPositionFromInitial;

      private final YoFrameVector positionError;
      private final YoFrameVector force;

      public GCPointMotionConstrainer(Robot robot, GroundContactPoint handGCPoint, double elapsedUnarrestedTime, double stiffness_NperM)
      {
         registry = new YoVariableRegistry(handGCPoint.getName() + "GCPointMotionConstrainer");
         this.robot = robot;
         this.handGCPoint = handGCPoint;

         this.elapsedUnarrestedTime = elapsedUnarrestedTime;
         this.stiffness_NperM = stiffness_NperM;

         allowableDirectionOfMotion = new YoFrameVector(handGCPoint.getName() + "AllowableDirectionOfMotion", ReferenceFrame.getWorldFrame(), registry);

         desiredPosition = new YoFramePoint(handGCPoint.getName() + "DesiredPosition", ReferenceFrame.getWorldFrame(), registry);
         initialPosition = new YoFramePoint(handGCPoint.getName() + "InitialPosition", ReferenceFrame.getWorldFrame(), registry);
         currentPosition = new YoFramePoint(handGCPoint.getName() + "CurrentPosition", ReferenceFrame.getWorldFrame(), registry);

         currentPositionFromInitial = new YoFrameVector(handGCPoint.getName() + "CurrentPosFromInitial", ReferenceFrame.getWorldFrame(), registry);
         desiredPositionFromInitial = new YoFrameVector(handGCPoint.getName() + "DesiredPosFromInitial", ReferenceFrame.getWorldFrame(), registry);

         positionError = new YoFrameVector(handGCPoint.getName() + "PositionError", ReferenceFrame.getWorldFrame(), registry);
         force = new YoFrameVector(handGCPoint.getName() + "Force", ReferenceFrame.getWorldFrame(), registry);

         initialize();
      }

      public void initialize()
      {
         allowableDirectionOfMotion.set(1.0, 0, 1.0);
         allowableDirectionOfMotion.normalize();
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public String getName()
      {
         return registry.getName();
      }

      public String getDescription()
      {
         return null;
      }

      boolean hasDoControlBeenCalled = false;

      public void doControl()
      {
         if (!hasDoControlBeenCalled)
         {
            desiredPosition.set(handGCPoint.getYoPosition());

            if (robot.getTime() > elapsedUnarrestedTime)
            {
               initialPosition.set(handGCPoint.getYoPosition());
               hasDoControlBeenCalled = true;
            }
         }
         else
         {
            currentPosition.set(handGCPoint.getYoPosition());
            currentPositionFromInitial.sub(currentPosition, initialPosition);

            double currentPositionProjectedOntoAllowableDirection = currentPositionFromInitial.dot(allowableDirectionOfMotion.getFrameTuple());

            desiredPositionFromInitial.set(allowableDirectionOfMotion.getFrameTuple());
            desiredPositionFromInitial.normalize();
            desiredPositionFromInitial.scale(currentPositionProjectedOntoAllowableDirection);

            desiredPosition.set(initialPosition);
            desiredPosition.add(desiredPositionFromInitial);

            positionError.sub(desiredPosition, currentPosition);

            force.set(positionError);
            force.scale(stiffness_NperM);

            handGCPoint.setForce(force.getFrameTuple().getVector());
         }
      }
   }

   private void setupCameraForHandstepsOnWalls()
   {
      Point3d cameraFix = new Point3d(1.8375, -0.16, 0.89);
      Point3d cameraPosition = new Point3d(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
