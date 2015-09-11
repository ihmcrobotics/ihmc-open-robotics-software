package us.ihmc.darpaRoboticsChallenge.behaviorTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWallWorldEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCBehaviorTestHelper;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCDetectCollisionUsingWristSensorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
      }
      
      if (updateForceSensorController != null)
      {
         updateForceSensorController = null;
      }
      
      if(handImpactDetector != null)
      {
         handImpactDetector = null;
      }
      
      if(wristGCPoint != null)
      {
         wristGCPoint = null;
      }

      GlobalTimer.clearTimers();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @AfterClass
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(DRCChestOrientationBehaviorTest.class + " after class.");
   }

   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private final RobotSide robotSideToTest = RobotSide.LEFT;

   private MonitorForceSensorController updateForceSensorController;
   private GroundContactPoint wristGCPoint;
   private HandImpactDetector handImpactDetector;

   @Before
   public void setUp()
   {
      DRCWallWorldEnvironment testEnvironment = new DRCWallWorldEnvironment(-0.5, 2.5);

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(testEnvironment, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, simulationTestingParameters, getRobotModel());

      updateForceSensorController = setupUpdateForceSensorController(drcBehaviorTestHelper.getRobotDataReceiver(),
            drcBehaviorTestHelper.getWristForceSensorUpdatable(robotSideToTest));
      drcBehaviorTestHelper.getRobot().setController(updateForceSensorController);

      wristGCPoint = drcBehaviorTestHelper.getRobot().getHandGroundContactPoints(robotSideToTest).get(0);

      handImpactDetector = new HandImpactDetector(drcBehaviorTestHelper.getRobot(), wristGCPoint);
      drcBehaviorTestHelper.getRobot().setController(handImpactDetector);
   }

   private MonitorForceSensorController setupUpdateForceSensorController(RobotDataReceiver robotDataReceiver,
         WristForceSensorFilteredUpdatable wristUpdatable)
   {
      ArrayList<WrenchCalculatorInterface> forceSensors = new ArrayList<WrenchCalculatorInterface>();
      drcBehaviorTestHelper.getRobot().getForceSensors(forceSensors);
      MonitorForceSensorController ret = new MonitorForceSensorController(wristUpdatable, forceSensors);
      return ret;
   }

   //   @Test(timeout=300000)
   public void testConstrainedHandPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      double timeToStartMotionConstraint = 1.3 * 1.0;
      double arrestStiffness_NperM = 10000.0;
      GCPointMotionConstrainer handGCArrestor = new GCPointMotionConstrainer(drcBehaviorTestHelper.getRobot(), wristGCPoint, timeToStartMotionConstraint,
            arrestStiffness_NperM);

      drcBehaviorTestHelper.getRobot().setController(handGCArrestor);

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);
      
      double trajectoryTime = 2.0;
      double handDeltaX = 0.4;
      double handDeltaY = 0.0;
      double handDeltaZ = 0.0;
      double handDeltaOmega = 0.0;

      HandPoseBehavior handPoseBehavior = createHandPoseBehavior(handDeltaX, handDeltaY, handDeltaZ, handDeltaOmega, trajectoryTime);
      
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(handPoseBehavior, trajectoryTime + 1.0);
      assertTrue(success);

      double actualHandGcCollisionTime = handImpactDetector.getHandImpactTime();
      double wristForceSensorCollisionTime = drcBehaviorTestHelper.getWristForceSensorUpdatable(robotSideToTest).getHandImpactTime();

      assertEquals("Expected time of detected impact = " + actualHandGcCollisionTime + ".  Actual detected impact time = " + wristForceSensorCollisionTime,
            actualHandGcCollisionTime, wristForceSensorCollisionTime, 0.1);

      BambooTools.reportTestFinishedMessage();
   }

   @DeployableTestMethod(estimatedDuration = 15.9)
   @Test(timeout = 47679)
   public void testImpactDetectionUsingHandPoseBehavior() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double trajectoryTime = 2.0;
      double handDeltaX = 0.25;
      double handDeltaY = 0.0;
      double handDeltaZ = 0.0;
      double handDeltaOmega = 0.0;

      HandPoseBehavior handPoseBehavior = createHandPoseBehavior(handDeltaX, handDeltaY, handDeltaZ, handDeltaOmega, trajectoryTime);
      
      success = drcBehaviorTestHelper.executeBehaviorUntilDone(handPoseBehavior);
      assertTrue(success);

      double actualHandGcCollisionTime = handImpactDetector.getHandImpactTime();
      double wristForceSensorCollisionTime = drcBehaviorTestHelper.getWristForceSensorUpdatable(robotSideToTest).getHandImpactTime();

      assertEquals("Expected time of detected impact = " + actualHandGcCollisionTime + ".  Actual detected impact time = " + wristForceSensorCollisionTime,
            actualHandGcCollisionTime, wristForceSensorCollisionTime, 0.1);

      BambooTools.reportTestFinishedMessage();
   }
   
   //FIXME: Running more than one test causes the following error: java.lang.RuntimeException: java.net.BindException: Address already in use: bind
   //      @Test(timeout=300000)
   public void testHandMassCompensationBySlowlyRotatingHands() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double trajectoryTime = 2.0;
      double handDeltaX = 0.4;
      double handDeltaY = 0.0;
      double handDeltaZ = 0.0;
      double handDeltaOmega = 0.0;

      HandPoseBehavior handPoseBehavior = createHandPoseBehavior(handDeltaX, handDeltaY, handDeltaZ, handDeltaOmega, trajectoryTime);
      
      success = drcBehaviorTestHelper.executeBehaviorSimulateAndBlockAndCatchExceptions(handPoseBehavior, trajectoryTime + 1.0);
      assertTrue(success);

      double maxWristForceMassCompensated = updateForceSensorController.getMaxWristForceMassCompensated();
      double reasonableMaxValue = 0.5 * Math.abs(9.81 * drcBehaviorTestHelper.getWristForceSensorUpdatable(robotSideToTest).getHandMass());

      System.out.println("MaxWristForceMassCompensated = " + maxWristForceMassCompensated + ".  Should be less than: " + reasonableMaxValue);

      Boolean maxWristForceMassCompensatedIsReasonablySmall = Math.abs(maxWristForceMassCompensated) < reasonableMaxValue;
      assertTrue(maxWristForceMassCompensatedIsReasonablySmall);

      BambooTools.reportTestFinishedMessage();
   }

   private final Quat4d handOrientation = new Quat4d();

   private HandPoseBehavior createHandPoseBehavior(double deltaX, double deltaY, double deltaZ, double deltaOmega, double swingTrajectoryTime)
   {
      final HandPoseBehavior ret = new HandPoseBehavior(drcBehaviorTestHelper.getBehaviorCommunicationBridge(), drcBehaviorTestHelper.getYoTime());

      final FramePose handPose = new FramePose();
      handPose.setToZero(drcBehaviorTestHelper.getSDFFullRobotModel().getHandControlFrame(robotSideToTest));

      handPose.changeFrame(ReferenceFrame.getWorldFrame());

      handPose.setX(handPose.getX() + deltaX);
      handPose.setY(handPose.getY() + deltaY);
      handPose.setZ(handPose.getZ() + deltaZ);

      handPose.getOrientation(handOrientation);
      handOrientation.setW(handOrientation.w + deltaOmega);
      handPose.setOrientation(handOrientation);

      final RigidBodyTransform handPoseTransform = new RigidBodyTransform();
      handPose.getPose(handPoseTransform);

      boolean stopHandPoseBehaviorIfCollisionIsDetected = true;
      
      ret.initialize();
      ret.setInput(Frame.WORLD, handPoseTransform, robotSideToTest, swingTrajectoryTime, stopHandPoseBehaviorIfCollisionIsDetected);
      
      return ret;
   }
   
   private class MonitorForceSensorController implements RobotController
   {
      private final YoVariableRegistry registry;
      private final WristForceSensorFilteredUpdatable wristUpdatable;
      private final DoubleYoVariable maxWristForceMassCompensated;

      public MonitorForceSensorController(WristForceSensorFilteredUpdatable wristUpdatable, ArrayList<WrenchCalculatorInterface> forceSensors)
      {
         registry = new YoVariableRegistry("ForceSensorController");
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
         double wristForceMassCompensated = wristUpdatable.getWristForceMassCompensatedInWorld().length();
         if (wristForceMassCompensated > maxWristForceMassCompensated.getDoubleValue() && drcBehaviorTestHelper.getYoTime().getDoubleValue() > 1.0)
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
}
