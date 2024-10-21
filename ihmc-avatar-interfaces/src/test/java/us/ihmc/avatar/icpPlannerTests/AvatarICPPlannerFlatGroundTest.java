package us.ihmc.avatar.icpPlannerTests;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
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
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class AvatarICPPlannerFlatGroundTest implements MultiRobotTestInterface
{
   private final static double defaultSwingTime = 0.6;
   private final static double defaultTransferTime = 2.5;
   private final static double defaultChickenPercentage = 0.5;

   private final static String chickenSupportName = "icpPlannerCoPTrajectoryGeneratorPercentageChickenSupport";

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SideDependentList<ArrayList<Point2D>> footContactsInAnkleFrame = null;

   private static SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
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

   /**
    * This test will drop the floor out from underneath the sim randomly while standing. Tests if
    * detection and hold position are working well.
    */
   @Disabled
   @Test
   public void testChangeOfSupport()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      Random random = new Random(1738L);

      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatEnvironment, simulationTestingParameters);
      simulationTestHelper.start();
      enablePartialFootholdDetectionAndResponse(simulationTestHelper);

      // Since the foot support points change while standing, the parts of the support polygon that need to be cut off might have had the CoP in them.
      YoBoolean useCoPOccupancyGrid = (YoBoolean) simulationTestHelper.findVariable("ExplorationFoothold_UseCopOccupancyGrid");
      useCoPOccupancyGrid.set(false);
      YoBoolean doFootExplorationInTransferToStanding = (YoBoolean) simulationTestHelper.findVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(false);

      YoDouble desiredICPX = (YoDouble) simulationTestHelper.findVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) simulationTestHelper.findVariable("desiredICPY");

      desiredICPX.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP X value is NaN.");
            }
         }
      });
      desiredICPY.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP Y value is NaN.");
            }
         }
      });

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.0);

      Robot robot = simulationTestHelper.getRobot();
      RobotSide robotSide = RobotSide.LEFT;
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      SideDependentList<String> jointNames = getFootJointNames(fullRobotModel);
      HighLevelHumanoidControllerToolbox controllerToolbox = simulationTestHelper.getHighLevelHumanoidControllerFactory()
                                                                                 .getHighLevelHumanoidControllerToolbox();

      int numberOfChanges = 4;

      for (int i = 0; i < numberOfChanges; i++)
      {
         ArrayList<Point2D> newContactPoints = generateContactPointsForHalfOfFoot(random, getRobotModel().getWalkingControllerParameters(), 0.4);
         changeAppendageGroundContactPointsToNewOffsets(simulationTestHelper.getSimulationTime(),
                                                        robot,
                                                        newContactPoints,
                                                        jointNames.get(robotSide),
                                                        robotSide);
         success = success & simulationTestHelper.simulateNow(2.0);
         if (!success)
            break;

         // check if the found support polygon is close to the actual one
         FrameConvexPolygon2DReadOnly foundSupport = controllerToolbox.getBipedSupportPolygons().getFootPolygonInSoleFrame(robotSide);
         FrameConvexPolygon2D actualSupport = new FrameConvexPolygon2D(foundSupport.getReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(newContactPoints));
         double epsilon = 5.0; // cm^2
         boolean close = Math.abs(foundSupport.getArea() - actualSupport.getArea()) * 10000 < epsilon;
         if (!close)
         {
            System.out.println("Area expected: " + actualSupport.getArea() * 10000 + " [cm^2]");
            System.out.println("Area found:    " + foundSupport.getArea() * 10000 + " [cm^2]");
         }
         assertTrue("Support polygon found does not match the actual one.", close);

         // step in place to reset robot
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), 0.0, 0.0, 0.0);
         newContactPoints = generateContactPointsForAllOfFoot();
         success = success && takeAStepOntoNewFootGroundContactPoints(simulationTestHelper.getSimulationTime(),
                                                                      robot,
                                                                      fullRobotModel,
                                                                      robotSide,
                                                                      newContactPoints,
                                                                      stepLocation,
                                                                      jointNames,
                                                                      true,
                                                                      defaultSwingTime,
                                                                      defaultTransferTime);
         if (!success)
            break;
      }

      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-0.06095496955280358, -0.001119333179390724, 0.7875020745919501);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test pauses walking after the first two steps to check that functionality, and then finishes
    * the plan.
    */
   @Test
   public void testPauseWalkingInSwing()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatEnvironment, simulationTestingParameters);
      simulationTestHelper.start();

      YoDouble desiredICPX = (YoDouble) simulationTestHelper.findVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) simulationTestHelper.findVariable("desiredICPY");

      desiredICPX.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP X value is NaN.");
            }
         }
      });
      desiredICPY.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP Y value is NaN.");
            }
         }
      });

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.0);

      int numberOfSteps = 5;
      double swingDuration = 1.0;
      double transferDuration = 0.3;
      FootstepDataListMessage message = createForwardWalkingFootsteps(numberOfSteps, 0.3, 0.3, swingDuration, transferDuration);
      simulationTestHelper.publishToController(message);

      simulationTestHelper.simulateNow(2 * (swingDuration + transferDuration));
      simulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(true));

      simulationTestHelper.simulateNow(3.0);
      simulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(false));

      simulationTestHelper.simulateNow((numberOfSteps) * (swingDuration + transferDuration));

      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test pauses walking on the first step to check that functionality, and then finishes the
    * plan.
    */
   @Test
   public void testPauseWalkingInTransferFirstStep()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatEnvironment, simulationTestingParameters);
      simulationTestHelper.start();

      YoDouble desiredICPX = (YoDouble) simulationTestHelper.findVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) simulationTestHelper.findVariable("desiredICPY");

      desiredICPX.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP X value is NaN.");
            }
         }
      });
      desiredICPY.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP Y value is NaN.");
            }
         }
      });

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.0);

      int numberOfSteps = 5;
      double swingDuration = 1.0;
      double transferDuration = 0.3;
      FootstepDataListMessage message = createForwardWalkingFootsteps(numberOfSteps, 0.3, 0.3, swingDuration, transferDuration);
      simulationTestHelper.publishToController(message);

      simulationTestHelper.simulateNow(0.8 * transferDuration);
      simulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(true));

      simulationTestHelper.simulateNow(4.0);
      simulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(false));

      simulationTestHelper.simulateNow((numberOfSteps + 1) * (swingDuration + transferDuration));

      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   /**
    * This test pauses walking after the first two steps to check that functionality, and then finishes
    * the plan.
    */
   @Test
   public void testPauseWalkingInTransfer()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatEnvironment = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatEnvironment, simulationTestingParameters);
      simulationTestHelper.start();

      YoDouble desiredICPX = (YoDouble) simulationTestHelper.findVariable("desiredICPX");
      YoDouble desiredICPY = (YoDouble) simulationTestHelper.findVariable("desiredICPY");

      desiredICPX.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP X value is NaN.");
            }
         }
      });
      desiredICPY.addListener(new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            if (Double.isNaN(v.getValueAsDouble()))
            {
               fail("Desired ICP Y value is NaN.");
            }
         }
      });

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(1.0);

      int numberOfSteps = 5;
      double swingDuration = 1.0;
      double transferDuration = 0.3;
      FootstepDataListMessage message = createForwardWalkingFootsteps(numberOfSteps, 0.3, 0.3, swingDuration, transferDuration);
      simulationTestHelper.publishToController(message);

      simulationTestHelper.simulateNow(2 * (swingDuration + transferDuration) + 0.8 * transferDuration);
      simulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(true));

      simulationTestHelper.simulateNow(4.0);
      simulationTestHelper.publishToController(HumanoidMessageTools.createPauseWalkingMessage(false));

      simulationTestHelper.simulateNow((numberOfSteps) * (swingDuration + transferDuration));

      //      simulationTestHelper.checkNothingChanged();

      assertTrue(success);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FootstepDataListMessage createForwardWalkingFootsteps(int numberOfSteps,
                                                                 double length,
                                                                 double stanceWidth,
                                                                 double swingDuration,
                                                                 double transferDuration)
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

   private void enablePartialFootholdDetectionAndResponse(SCS2AvatarTestingSimulation simulationTestHelper)
   {
      enablePartialFootholdDetectionAndResponse(simulationTestHelper, defaultChickenPercentage);
   }

   private void enablePartialFootholdDetectionAndResponse(SCS2AvatarTestingSimulation simulationTestHelper, double chickenPercentage)
   {
      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         String footName = fullRobotModel.getFoot(robotSide).getName();
         YoBoolean doPartialFootholdDetection = (YoBoolean) simulationTestHelper.findVariable(footName + "DoPartialFootholdDetection");
         doPartialFootholdDetection.set(true);
      }

      YoBoolean doFootExplorationInTransferToStanding = (YoBoolean) simulationTestHelper.findVariable("doFootExplorationInTransferToStanding");
      doFootExplorationInTransferToStanding.set(true);

      YoDouble percentageChickenSupport = (YoDouble) simulationTestHelper.findVariable(chickenSupportName);
      percentageChickenSupport.set(chickenPercentage);

      YoDouble timeBeforeExploring = (YoDouble) simulationTestHelper.findVariable("ExplorationState_TimeBeforeExploring");
      timeBeforeExploring.set(0.0);
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private boolean takeAStepOntoNewFootGroundContactPoints(double time,
                                                           Robot robot,
                                                           FullHumanoidRobotModel fullRobotModel,
                                                           RobotSide robotSide,
                                                           ArrayList<Point2D> contactPointsInAnkleFrame,
                                                           FramePoint3D placeToStep,
                                                           SideDependentList<String> jointNames,
                                                           boolean setPredictedContactPoints,
                                                           double swingTime,
                                                           double transferTime)
   {
      return takeAStepOntoNewFootGroundContactPoints(time,
                                                     robot,
                                                     fullRobotModel,
                                                     robotSide,
                                                     contactPointsInAnkleFrame,
                                                     contactPointsInAnkleFrame,
                                                     placeToStep,
                                                     jointNames,
                                                     setPredictedContactPoints,
                                                     swingTime,
                                                     transferTime);
   }

   private boolean takeAStepOntoNewFootGroundContactPoints(double time,
                                                           Robot robot,
                                                           FullHumanoidRobotModel fullRobotModel,
                                                           RobotSide robotSide,
                                                           ArrayList<Point2D> contactPointsInAnkleFrame,
                                                           ArrayList<Point2D> predictedContactPointsInAnkleFrame,
                                                           FramePoint3D placeToStep,
                                                           SideDependentList<String> jointNames,
                                                           boolean setPredictedContactPoints,
                                                           double swingTime,
                                                           double transferTime)
   {
      String jointName = jointNames.get(robotSide);

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      FootstepDataMessage footstepData = createFootstepDataMessage(fullRobotModel,
                                                                   robotSide,
                                                                   predictedContactPointsInAnkleFrame,
                                                                   placeToStep,
                                                                   setPredictedContactPoints);
      message.getFootstepDataList().add().set(footstepData);

      simulationTestHelper.publishToController(message);
      boolean success = simulationTestHelper.simulateNow(1.2);
      changeAppendageGroundContactPointsToNewOffsets(time, robot, contactPointsInAnkleFrame, jointName, robotSide);
      success = success && simulationTestHelper.simulateNow(2.0);

      return success;
   }

   private FootstepDataMessage createFootstepDataMessage(FullHumanoidRobotModel fullRobotModel,
                                                         RobotSide robotSide,
                                                         ArrayList<Point2D> contactPointsInAnkleFrame,
                                                         FramePoint3D placeToStep,
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

   private void changeAppendageGroundContactPointsToNewOffsets(double time,
                                                               Robot robot,
                                                               ArrayList<Point2D> newContactPoints,
                                                               String jointName,
                                                               RobotSide robotSide)
   {
      System.out.println("Changing contact points at time " + time);

      int pointIndex = 0;
      List<GroundContactPoint> allGroundContactPoints = new ArrayList<>();
      for (SimJointBasics joint : robot.getAllJoints())
      {
         allGroundContactPoints.addAll(joint.getAuxialiryData().getGroundContactPoints());
      }

      for (GroundContactPoint point : allGroundContactPoints)
      {
         if (point.getParentJoint().getName().equals(jointName))
         {
            Point2D newContactPoint = newContactPoints.get(pointIndex);

            point.getInContact().set(false);
            point.getOffset().getPosition().setX(newContactPoint.getX());
            point.getOffset().getPosition().setY(newContactPoint.getY());
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
      double widthAtBack = percentToKeep * footWidth + (1 - percentToKeep) * toeWidth;
      ret.add(new Point2D(footForwardOffset, toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset, -toeWidth / 2.0));
      ret.add(new Point2D(footForwardOffset - length, -widthAtBack / 2.0));
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
      double widthAtFront = percentToKeep * toeWidth + (1 - percentToKeep) * footWidth;
      ret.add(new Point2D(-footBackwardOffset + length, widthAtFront / 2.0));
      ret.add(new Point2D(-footBackwardOffset + length, -widthAtFront / 2.0));
      ret.add(new Point2D(-footBackwardOffset, -footWidth / 2.0));
      ret.add(new Point2D(-footBackwardOffset, footWidth / 2.0));
      return ret;
   }
}
