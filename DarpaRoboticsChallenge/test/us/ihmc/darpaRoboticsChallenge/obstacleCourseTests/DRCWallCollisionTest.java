package us.ihmc.darpaRoboticsChallenge.obstacleCourseTests;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.MultiRobotTestInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCWallWorldEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidBehaviors.utilities.WristForceSensorFilteredUpdatable;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.net.LocalObjectCommunicator;
import us.ihmc.utilities.net.ObjectCommunicator;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public abstract class DRCWallCollisionTest implements MultiRobotTestInterface
{
   private static final boolean KEEP_SCS_UP = true;

   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean checkNothingChanged = BambooTools.getCheckNothingChanged();
   private static final boolean showGUI = KEEP_SCS_UP || createMovie;

   private DRCWallWorldEnvironment testEnvironment = new DRCWallWorldEnvironment(-0.5, 2.5);
   private ObjectCommunicator networkObjectCommunicator = new LocalObjectCommunicator();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private RobotSide robotSideToTest = RobotSide.LEFT;

   private String forceSensorNameSuffix = "wrx"; //TODO: Fix this so that it works with robots other than Atlas, by using DRCRobotSensorInformation.getWristForceSensorNames()
   private GroundContactPoint wristGCPoint;

   private ForceSensorDataHolder forceSensorDataHolder;
   private RobotDataReceiver robotDataReceiver;
   private WristForceSensorFilteredUpdatable wristUpdatable;
   private UpdateForceSensorFromRobotController updateForceSensorController;
   private HandGCArrestor handGCArrestor;

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      drcSimulationTestHelper = new DRCSimulationTestHelper(testEnvironment, networkObjectCommunicator, getSimpleRobotName(), null,
            DRCObstacleCourseStartingLocation.DEFAULT, checkNothingChanged, showGUI, createMovie, true, getRobotModel());

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      FullRobotModel fullRobotModel = getRobotModel().createFullRobotModel();

      wristGCPoint = setupHandGCPoint();

      forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullRobotModel.getForceSensorDefinitions()));

      robotDataReceiver = new RobotDataReceiver(fullRobotModel, forceSensorDataHolder);
      networkObjectCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);

      wristUpdatable = setupWristForceSensorUpdatable(forceSensorDataHolder);

      updateForceSensorController = setupUpdateForceSensorController(robotDataReceiver);
      robotToTest.setController(updateForceSensorController);

      handGCArrestor = setupHandGroundContactPointArrestor();
      robotToTest.setController(handGCArrestor);

      setupCameraForHandstepsOnWalls();
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
      {
         ThreadTools.sleepForever();
      }

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void sendHandPosePacket(HandPosePacket handPosePacket)
   {
      networkObjectCommunicator.consumeObject(handPosePacket);
   }

   private GroundContactPoint setupHandGCPoint()
   {
      GroundContactPoint ret = null;

      for (GroundContactPoint gcPoint : drcSimulationTestHelper.getRobot().getAllGroundContactPoints())
      {
         if (gcPoint.getName().contains(forceSensorNameSuffix) && gcPoint.getName().contains(robotSideToTest.getShortLowerCaseName()))
         {
            ret = gcPoint;
         }
      }

      return ret;
   }

   private WristForceSensorFilteredUpdatable setupWristForceSensorUpdatable(ForceSensorDataHolder forceSensorDataHolder)
   {
      Robot robotToTest = drcSimulationTestHelper.getRobot();
      YoVariableRegistry robotYoVariableRegistry = robotToTest.getRobotsYoVariableRegistry();

      FullRobotModel fullRobotModel = getRobotModel().createFullRobotModel();

      double DT = drcSimulationTestHelper.getSimulationConstructionSet().getDT();
      WristForceSensorFilteredUpdatable ret = new WristForceSensorFilteredUpdatable(robotSideToTest, fullRobotModel, forceSensorDataHolder, DT,
            networkObjectCommunicator, robotYoVariableRegistry);

      return ret;
   }

   private UpdateForceSensorFromRobotController setupUpdateForceSensorController(RobotDataReceiver robotDataReceiver)
   {
      ArrayList<WrenchCalculatorInterface> forceSensors = new ArrayList<WrenchCalculatorInterface>();

      Robot robotToTest = drcSimulationTestHelper.getRobot();
      robotToTest.getForceSensors(forceSensors);

      UpdateForceSensorFromRobotController ret = new UpdateForceSensorFromRobotController(robotToTest, robotDataReceiver, wristUpdatable, forceSensors);

      return ret;

   }

   @Test
   public void testImpactDetection() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      wristUpdatable.setStopMotionIfCollision(true);

      Point3d position = new Point3d(0.9, 0.35, 1.0);
      Quat4d orientation = new Quat4d(-1.0, 0.0, 0.0, 0.5 * Math.PI);
      double swingTrajectoryTime = 2.0;

      HandPosePacket handCollisionWithWallPosePacket = new HandPosePacket(robotSideToTest, Frame.CHEST, position, orientation, swingTrajectoryTime);
      sendHandPosePacket(handCollisionWithWallPosePacket);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);
      
      double actualHandGcCollisionTime = handGCArrestor.getHandGcCollisionTime();
      double wristForceSensorCollisionTime = wristUpdatable.getHandImpactTime();

      assertEquals("Expected time of detected impact = " + actualHandGcCollisionTime + ".  Actual detected impact time = " + wristForceSensorCollisionTime,
            actualHandGcCollisionTime, wristForceSensorCollisionTime, 0.1);

      BambooTools.reportTestFinishedMessage();
   }

   //FIXME: Running more than one test causes the following error: java.lang.RuntimeException: java.net.BindException: Address already in use: bind
   //      @Test
   public void testHandMassCompensationBySlowlyRotatingHands() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      wristUpdatable.setStopMotionIfCollision(false);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      Point3d position = new Point3d(0.9 / 2.5, 0.0, 1.0 * 0.8);
      Quat4d orientation = new Quat4d(-1.0, 0.0, 0.0, 0.0);
      double swingTrajectoryTime = 4.0;

      HandPosePacket slowArmRotationHandPosePacket = new HandPosePacket(robotSideToTest, Frame.CHEST, position, orientation, swingTrajectoryTime);
      sendHandPosePacket(slowArmRotationHandPosePacket);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
      assertTrue(success);

      double maxWristForceMassCompensated = updateForceSensorController.getMaxWristForceMassCompensated();
      double reasonableMaxValue = 0.5 * Math.abs(9.81 * wristUpdatable.getHandMass());

      //      System.out.println("MaxWristForceMassCompensated = " + maxWristForceMassCompensated + ".  Should be less than: " + reasonableMaxValue);

      Boolean maxWristForceMassCompensatedIsReasonablySmall = Math.abs(maxWristForceMassCompensated) < reasonableMaxValue;
      assertTrue(maxWristForceMassCompensatedIsReasonablySmall);

      BambooTools.reportTestFinishedMessage();
   }

   private HandGCArrestor setupHandGroundContactPointArrestor()
   {
      Robot robotToTest = drcSimulationTestHelper.getRobot();

      double timeToArrest = 999.0;
      double arrestStiffness_NperM = 10000.0;
      HandGCArrestor ret = new HandGCArrestor(robotToTest, wristGCPoint, timeToArrest, arrestStiffness_NperM);

      return ret;
   }

   private class UpdateForceSensorFromRobotController implements RobotController
   {
      private final YoVariableRegistry registry;
      private final Robot robot;
      private final RobotDataReceiver robotDataReceiver;
      private final WristForceSensorFilteredUpdatable wristUpdatable;
      private final DoubleYoVariable maxWristForceMassCompensated;

      public UpdateForceSensorFromRobotController(Robot robot, RobotDataReceiver robotDataReceiver, WristForceSensorFilteredUpdatable wristUpdatable,
            ArrayList<WrenchCalculatorInterface> forceSensors)
      {
         registry = new YoVariableRegistry("ForceSensorController");
         this.robot = robot;
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
         wristUpdatable.update(robot.getTime());

         double wristForceMassCompensated = wristUpdatable.getWristForceMassCompensated().length();
         if (wristForceMassCompensated > maxWristForceMassCompensated.getDoubleValue() && robot.getTime() > 1.0)
         {
            maxWristForceMassCompensated.set(wristForceMassCompensated);
         }
      }
   }

   private class HandGCArrestor implements RobotController
   {
      private final YoVariableRegistry registry;
      private final Robot robot;
      private final GroundContactPoint handGCPoint;

      private final double elapsedUnarrestedTime;
      private final double stiffness_NperM;

      private final DoubleYoVariable handGCPointCollisionTime;

      private final YoFramePoint desiredPosition;
      private final YoFramePoint currentPosition;
      private final YoFrameVector positionError;
      private final YoFrameVector force;

      public HandGCArrestor(Robot robot, GroundContactPoint handGCPoint, double elapsedUnarrestedTime, double stiffness_NperM)
      {
         registry = new YoVariableRegistry(handGCPoint.getName() + "HandGCArrestor");
         this.robot = robot;
         this.handGCPoint = handGCPoint;

         this.elapsedUnarrestedTime = elapsedUnarrestedTime;
         this.stiffness_NperM = stiffness_NperM;

         handGCPointCollisionTime = new DoubleYoVariable(handGCPoint.getName() + "CollisionTime", registry);

         desiredPosition = new YoFramePoint(handGCPoint.getName() + "DesiredPosition", ReferenceFrame.getWorldFrame(), registry);
         currentPosition = new YoFramePoint(handGCPoint.getName() + "CurrentPosition", ReferenceFrame.getWorldFrame(), registry);
         positionError = new YoFrameVector(handGCPoint.getName() + "PositionError", ReferenceFrame.getWorldFrame(), registry);
         force = new YoFrameVector(handGCPoint.getName() + "Force", ReferenceFrame.getWorldFrame(), registry);
      }

      public double getHandGcCollisionTime()
      {
         return handGCPointCollisionTime.getDoubleValue();
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
         return null;
      }

      boolean hasDoControlBeenCalled = false;
      private final Point3d temp = new Point3d();

      boolean handGCHasCollided = false;

      public void doControl()
      {
         if (handGCPoint.isInContact() && !handGCHasCollided)
         {
            handGCPointCollisionTime.set(robot.getTime());
            handGCHasCollided = true;
         }

         if (!hasDoControlBeenCalled)
         {
            handGCPoint.getPosition(temp);
            desiredPosition.set(temp);

            if (robot.getTime() > elapsedUnarrestedTime)
            {
               hasDoControlBeenCalled = true;
            }
         }
         else
         {
            handGCPoint.getPosition(temp);
            currentPosition.set(temp);
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
