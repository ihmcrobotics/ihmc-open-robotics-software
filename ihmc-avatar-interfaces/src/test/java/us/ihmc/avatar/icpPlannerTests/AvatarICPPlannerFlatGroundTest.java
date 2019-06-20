package us.ihmc.avatar.icpPlannerTests;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

@Tag("humanoid-flat-ground")
public abstract class AvatarICPPlannerFlatGroundTest implements MultiRobotTestInterface
{
   private final static double defaultSwingTime = 0.6;
   private final static double defaultTransferTime = 2.5;
   private final static double defaultChickenPercentage = 0.5;

   private final static String chickenSupportName = "icpPlannerCoPTrajectoryGeneratorPercentageChickenSupport";

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SideDependentList<ArrayList<Point2D>> footContactsInAnkleFrame = null;

   private static SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   }

   @AfterEach
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

   /**
    * This test will drop the floor out from underneath the sim randomly while standing. Tests if detection and hold position are working well.
    */
   @Disabled
   @Test
   public void testChangeOfSupport() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      Random random = new Random(1738L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatEnvironment);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("ICPFlatGroundTest");
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper);

      // Since the foot support points change while standing, the parts of the support polygon that need to be cut off might have had the CoP in them.
      YoBoolean useCoPOccupancyGrid = (YoBoolean) drcSimulationTestHelper.getYoVariable("ExplorationFoothold_UseCopOccupancyGrid");
      useCoPOccupancyGrid.set(false);
      YoBoolean doFootExplorationInTransferToStanding = (YoBoolean) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(false);

      YoDouble desiredICPX = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPY");

      desiredICPX.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP X value is NaN.");
            }
         }
      });
      desiredICPY.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP Y value is NaN.");
            }
         }
      });

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      RobotSide robotSide = RobotSide.LEFT;
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);
      HighLevelHumanoidControllerToolbox controllerToolbox = drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory()
                                                                                          .getHighLevelHumanoidControllerToolbox();

      int numberOfChanges = 4;

      for (int i=0; i<numberOfChanges; i++)
      {
         ArrayList<Point2D> newContactPoints = generateContactPointsForHalfOfFoot(random, getRobotModel().getWalkingControllerParameters(), 0.4);
         changeAppendageGroundContactPointsToNewOffsets(robot, newContactPoints, jointNames.get(robotSide), robotSide);
         success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
         if (!success) break;

         // check if the found support polygon is close to the actual one
         FrameConvexPolygon2DReadOnly foundSupport = controllerToolbox.getBipedSupportPolygons().getFootPolygonInSoleFrame(robotSide);
         FrameConvexPolygon2D actualSupport = new FrameConvexPolygon2D(foundSupport.getReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(newContactPoints));
         double epsilon = 5.0; // cm^2
         boolean close = Math.abs(foundSupport.getArea() - actualSupport.getArea()) * 10000 < epsilon;
         if (!close) {
            System.out.println("Area expected: " + actualSupport.getArea()*10000 + " [cm^2]");
            System.out.println("Area found:    " + foundSupport.getArea()*10000  + " [cm^2]");
         }
         assertTrue("Support polygon found does not match the actual one.", close);

         // step in place to reset robot
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), 0.0, 0.0, 0.0);
         newContactPoints = generateContactPointsForAllOfFoot();
         success = success && takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, newContactPoints, stepLocation, jointNames, true, defaultSwingTime, defaultTransferTime);
         if (!success) break;
      }

      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-0.06095496955280358, -0.001119333179390724, 0.7875020745919501);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test pauses walking after the first two steps to check that functionality, and then finishes the plan.
    */
   @Test
   public void testPauseWalkingInSwing() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatEnvironment);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("ICPFlatGroundTest");

      YoDouble desiredICPX = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPY");

      desiredICPX.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP X value is NaN.");
            }
         }
      });
      desiredICPY.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP Y value is NaN.");
            }
         }
      });


      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 5;
      double swingDuration = 1.0;
      double transferDuration = 0.3;
      FootstepDataListMessage message = createForwardWalkingFootsteps(numberOfSteps, 0.3, 0.3, swingDuration, transferDuration);
      drcSimulationTestHelper.publishToController(message);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2 * (swingDuration + transferDuration));
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(true));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(false));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions((numberOfSteps) * (swingDuration + transferDuration));

      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test pauses walking on the first step to check that functionality, and then finishes the plan.
    */
   @Test
   public void testPauseWalkingInTransferFirstStep() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatEnvironment);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("ICPFlatGroundTest");

      YoDouble desiredICPX = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPY");

      desiredICPX.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP X value is NaN.");
            }
         }
      });
      desiredICPY.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP Y value is NaN.");
            }
         }
      });


      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 5;
      double swingDuration = 1.0;
      double transferDuration = 0.3;
      FootstepDataListMessage message = createForwardWalkingFootsteps(numberOfSteps, 0.3, 0.3, swingDuration, transferDuration);
      drcSimulationTestHelper.publishToController(message);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.8 * transferDuration);
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(true));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(false));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions((numberOfSteps + 1) * (swingDuration + transferDuration));

      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test pauses walking after the first two steps to check that functionality, and then finishes the plan.
    */
   @Test
   public void testPauseWalkingInTransfer() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatEnvironment);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("ICPFlatGroundTest");

      YoDouble desiredICPX = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) drcSimulationTestHelper.getYoVariable("desiredICPY");

      desiredICPX.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP X value is NaN.");
            }
         }
      });
      desiredICPY.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP Y value is NaN.");
            }
         }
      });


      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      int numberOfSteps = 5;
      double swingDuration = 1.0;
      double transferDuration = 0.3;
      FootstepDataListMessage message = createForwardWalkingFootsteps(numberOfSteps, 0.3, 0.3, swingDuration, transferDuration);
      drcSimulationTestHelper.publishToController(message);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2 * (swingDuration + transferDuration) + 0.8 * transferDuration);
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(true));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(false));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions((numberOfSteps) * (swingDuration + transferDuration));

      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   private FootstepDataListMessage createForwardWalkingFootsteps(int numberOfSteps, double length, double stanceWidth, double swingDuration, double transferDuration)
   {
      FootstepDataListMessage footstepListMessage = new FootstepDataListMessage();

      double xLocation = 0.0;
      RobotSide robotSide = RobotSide.LEFT;
      for (int i = 0; i < numberOfSteps - 1; i++)
      {
         xLocation += length;
         FootstepDataMessage footstepMessage = new FootstepDataMessage();
         footstepMessage.setSwingDuration(swingDuration);
         footstepMessage.setTransferDuration(transferDuration);
         footstepMessage.getLocation().set(new Point3D(xLocation, robotSide.negateIfRightSide(stanceWidth / 2.0), 0.0));
         footstepMessage.getOrientation().set(new Quaternion());
         footstepMessage.setRobotSide(robotSide.toByte());

         footstepListMessage.getFootstepDataList().add().set(footstepMessage);
         robotSide = robotSide.getOppositeSide();
      }

      FootstepDataMessage footstepMessage = new FootstepDataMessage();
      footstepMessage.setSwingDuration(swingDuration);
      footstepMessage.setTransferDuration(transferDuration);
      footstepMessage.getLocation().set(new Point3D(xLocation, robotSide.negateIfRightSide(stanceWidth / 2.0), 0.0));
      footstepMessage.getOrientation().set(new Quaternion());
      footstepMessage.setRobotSide(robotSide.toByte());

      footstepListMessage.getFootstepDataList().add().set(footstepMessage);
      footstepListMessage.setDefaultSwingDuration(swingDuration);
      footstepListMessage.setDefaultTransferDuration(transferDuration);

      return footstepListMessage;
   }

   private SideDependentList<String> getFootJointNames(FullHumanoidRobotModel fullRobotModel)
   {
      SideDependentList<String> jointNames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         jointNames.put(robotSide, fullRobotModel.getFoot(robotSide).getParentJoint().getName());
      }
      return jointNames;
   }

   private void enablePartialFootholdDetectionAndResponse(DRCSimulationTestHelper drcSimulationTestHelper)
   {
      enablePartialFootholdDetectionAndResponse(drcSimulationTestHelper, defaultChickenPercentage);
   }

   private void enablePartialFootholdDetectionAndResponse(DRCSimulationTestHelper drcSimulationTestHelper, double chickenPercentage)
   {
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidFloatingRootJointRobot simulatedRobot = drcSimulationTestHelper.getAvatarSimulation().getHumanoidFloatingRootJointRobot();

      for (RobotSide robotSide : RobotSide.values)
      {
         String footName = fullRobotModel.getFoot(robotSide).getName();
         YoBoolean doPartialFootholdDetection = (YoBoolean) drcSimulationTestHelper.getYoVariable(footName + "DoPartialFootholdDetection");
         doPartialFootholdDetection.set(true);

         // Set joint velocity limits on the ankle joints so that they don't flip out when doing partial footsteps. On real robots with joint velocity limits, this should
         // happen naturally.
         double qd_max = 12.0;
         double b_vel_limit = 500.0;

         String firstAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getName();
         if (simulatedRobot.getOneDegreeOfFreedomJoint(firstAnkleName) instanceof PinJoint)
         {
            PinJoint ankleJoint = (PinJoint) simulatedRobot.getOneDegreeOfFreedomJoint(firstAnkleName);
            ankleJoint.setVelocityLimits(qd_max , b_vel_limit);
         }
         else
         {
            throw new RuntimeException("Can not set velocity limits on ankle joint " + firstAnkleName + " - it is not a PinJoint.");
         }

         String secondAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getName();
         if (simulatedRobot.getOneDegreeOfFreedomJoint(secondAnkleName) instanceof PinJoint)
         {
            PinJoint ankleJoint = (PinJoint) simulatedRobot.getOneDegreeOfFreedomJoint(secondAnkleName);
            ankleJoint.setVelocityLimits(qd_max , b_vel_limit);
         }
         else
         {
            throw new RuntimeException("Can not set velocity limits on ankle joint " + secondAnkleName + " - it is not a PinJoint.");
         }
      }

      YoBoolean doFootExplorationInTransferToStanding = (YoBoolean) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(true);

      YoDouble percentageChickenSupport = (YoDouble) drcSimulationTestHelper.getYoVariable(chickenSupportName);
      percentageChickenSupport.set(chickenPercentage);

      YoDouble timeBeforeExploring = (YoDouble) drcSimulationTestHelper.getYoVariable("ExplorationState_TimeBeforeExploring");
      timeBeforeExploring.set(0.0);
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }


   private boolean takeAStepOntoNewFootGroundContactPoints(HumanoidFloatingRootJointRobot robot, FullHumanoidRobotModel fullRobotModel, RobotSide robotSide,
         ArrayList<Point2D> contactPointsInAnkleFrame, FramePoint3D placeToStep, SideDependentList<String> jointNames, boolean setPredictedContactPoints, double swingTime, double transferTime)
         throws SimulationExceededMaximumTimeException
   {
      return takeAStepOntoNewFootGroundContactPoints(robot, fullRobotModel, robotSide, contactPointsInAnkleFrame, contactPointsInAnkleFrame, placeToStep,
            jointNames, setPredictedContactPoints, swingTime, transferTime);
   }

   private boolean takeAStepOntoNewFootGroundContactPoints(HumanoidFloatingRootJointRobot robot, FullHumanoidRobotModel fullRobotModel, RobotSide robotSide,
         ArrayList<Point2D> contactPointsInAnkleFrame, ArrayList<Point2D> predictedContactPointsInAnkleFrame, FramePoint3D placeToStep,
         SideDependentList<String> jointNames, boolean setPredictedContactPoints, double swingTime, double transferTime) throws SimulationExceededMaximumTimeException
   {
      String jointName = jointNames.get(robotSide);

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      FootstepDataMessage footstepData = createFootstepDataMessage(fullRobotModel, robotSide, predictedContactPointsInAnkleFrame, placeToStep, setPredictedContactPoints);
      message.getFootstepDataList().add().set(footstepData);

      drcSimulationTestHelper.publishToController(message);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.2);
      changeAppendageGroundContactPointsToNewOffsets(robot, contactPointsInAnkleFrame, jointName, robotSide);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      return success;
   }

   private FootstepDataMessage createFootstepDataMessage(FullHumanoidRobotModel fullRobotModel, RobotSide robotSide, ArrayList<Point2D> contactPointsInAnkleFrame, FramePoint3D placeToStep,
         boolean setPredictedContactPoints)
   {
      ReferenceFrame ankleFrame = fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG);
      ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);

      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint3D placeToStepInWorld = new FramePoint3D(placeToStep);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.getLocation().set(placeToStepInWorld);
      footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide.toByte());

      if (setPredictedContactPoints && (contactPointsInAnkleFrame != null))
      {
         ArrayList<Point2D> contactPointsInSoleFrame = transformFromAnkleFrameToSoleFrame(contactPointsInAnkleFrame, ankleFrame, soleFrame);
         HumanoidMessageTools.packPredictedContactPoints(contactPointsInSoleFrame, footstepData);
      }
      return footstepData;
   }

   private void changeAppendageGroundContactPointsToNewOffsets(HumanoidFloatingRootJointRobot robot, ArrayList<Point2D> newContactPoints, String jointName, RobotSide robotSide)
   {
      double time = robot.getTime();
      System.out.println("Changing contact points at time " + time);

      int pointIndex = 0;
      ArrayList<GroundContactPoint> allGroundContactPoints = robot.getAllGroundContactPoints();

      for (GroundContactPoint point : allGroundContactPoints)
      {
         Joint parentJoint = point.getParentJoint();

         if (parentJoint.getName().equals(jointName))
         {
            Point2D newContactPoint = newContactPoints.get(pointIndex);

            point.setIsInContact(false);
            Vector3D offset = new Vector3D();
            point.getOffset(offset);
            //            System.out.println("originalOffset = " + offset);

            offset.setX(newContactPoint.getX());
            offset.setY(newContactPoint.getY());

            //            System.out.println("newOffset = " + offset);
            point.setOffsetJoint(offset);

            pointIndex++;
         }
      }

      if (footContactsInAnkleFrame != null)
      {
         footContactsInAnkleFrame.set(robotSide, newContactPoints);
      }
   }

   private ArrayList<Point2D> transformFromAnkleFrameToSoleFrame(ArrayList<Point2D> originalPoints, ReferenceFrame ankleFrame, ReferenceFrame soleFrame)
   {
      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      for (Point2D originalPoint : originalPoints)
      {
         FramePoint3D framePoint = new FramePoint3D(ankleFrame, originalPoint.getX(), originalPoint.getY(), 0.0);
         framePoint.changeFrame(soleFrame);

         ret.add(new Point2D(framePoint.getX(), framePoint.getY()));
      }

      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForAllOfFoot()
   {
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      double toeWidth = walkingControllerParameters.getSteppingParameters().getToeWidth();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      ret.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForHalfOfFoot(Random random, WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      int footHalf = random.nextInt(4);

      if (footHalf == 0)
         return generateContactPointsForLeftOfFoot(walkingControllerParameters, percentToKeep);
      if (footHalf == 1)
         return generateContactPointsForRightOfFoot(walkingControllerParameters, percentToKeep);
      if (footHalf == 2)
         return generateContactPointsForFrontOfFoot(walkingControllerParameters, percentToKeep);

      return generateContactPointsForBackOfFoot(walkingControllerParameters, percentToKeep);
   }

   private ArrayList<Point2D> generateContactPointsForLeftOfFoot(WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      double toeWidth = walkingControllerParameters.getSteppingParameters().getToeWidth();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      double width = percentToKeep * (toeWidth + footWidth) / 2.0;
      ret.add(new Point2D(footForwardOffset, width - toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, 0.0));
      ret.add(new Point2D(-footBackwardOffset, 0.0));
      ret.add(new Point2D(-footBackwardOffset, width - footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForRightOfFoot(WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      double toeWidth = walkingControllerParameters.getSteppingParameters().getToeWidth();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      double width = percentToKeep * (toeWidth + footWidth) / 2.0;
      ret.add(new Point2D(footForwardOffset, width - toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, width - footWidth / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForFrontOfFoot(WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      double footForwardOffset = walkingControllerParameters.getSteppingParameters().getFootForwardOffset();
      double footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      double toeWidth = walkingControllerParameters.getSteppingParameters().getToeWidth();
      double footLength = walkingControllerParameters.getSteppingParameters().getFootLength();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      double length = percentToKeep * (footLength);
      double widthAtBack = percentToKeep * footWidth + (1-percentToKeep) * toeWidth;
      ret.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, - toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset - length, - widthAtBack / 2.0));
      ret.add(new Point2D(footForwardOffset - length, widthAtBack / 2.0));
      return ret;
   }

   private ArrayList<Point2D> generateContactPointsForBackOfFoot(WalkingControllerParameters walkingControllerParameters, double percentToKeep)
   {
      double footBackwardOffset = walkingControllerParameters.getSteppingParameters().getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getSteppingParameters().getFootWidth();
      double toeWidth = walkingControllerParameters.getSteppingParameters().getToeWidth();
      double footLength = walkingControllerParameters.getSteppingParameters().getFootLength();

      ArrayList<Point2D> ret = new ArrayList<Point2D>();

      double length = percentToKeep * (footLength);
      double widthAtFront = percentToKeep * toeWidth + (1-percentToKeep) * footWidth;
      ret.add(new Point2D(-footBackwardOffset + length, widthAtFront / 2.0));
      ret.add(new Point2D(-footBackwardOffset + length, - widthAtFront / 2.0));
      ret.add(new Point2D(-footBackwardOffset, - footWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }
}
