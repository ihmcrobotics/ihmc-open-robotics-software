package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExploreFootPolygonState.ExplorationMethod;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.communication.packets.TrajectoryPoint1DMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public abstract class HumanoidLineContactWalkingTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private OffsetAndYawRobotInitialSetup location = new OffsetAndYawRobotInitialSetup(new Vector3d(0.0, 0.0, 0.0), 0.0);
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private final YoVariableRegistry registry = new YoVariableRegistry("PointyRocksTest");
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SideDependentList<YoFrameConvexPolygon2d> supportPolygons = null;
   private SideDependentList<ArrayList<Point2d>> footContactsInAnkleFrame = null;

   private ContactPointController contactPointController;
   private SideDependentList<EnumYoVariable<ConstraintType>> footStates = new SideDependentList<>();

   private BooleanYoVariable doFootExplorationInTransferToStanding;
   private DoubleYoVariable transferTime;
   private DoubleYoVariable swingTime;
   private DoubleYoVariable percentageChickenSupport;
   private DoubleYoVariable timeBeforeExploring;
   private SideDependentList<BooleanYoVariable> autoCropToLineAfterExploration = new SideDependentList<>();
   private SideDependentList<BooleanYoVariable> holdFlatDuringExploration = new SideDependentList<>();
   private SideDependentList<BooleanYoVariable> holdFlatDuringHoldPosition = new SideDependentList<>();
   private SideDependentList<BooleanYoVariable> smartHoldPosition = new SideDependentList<>();
   private SideDependentList<EnumYoVariable<ExplorationMethod>> explorationMethods = new SideDependentList<>();
   private BooleanYoVariable allowUpperBodyMomentumInSingleSupport;
   private BooleanYoVariable allowUpperBodyMomentumInDoubleSupport;
   private BooleanYoVariable allowUsingHighMomentumWeight;
   private BooleanYoVariable doToeOffIfPossible;

   @ContinuousIntegrationTest(estimatedDuration = 100.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 300000)
   public void testWalkingOnStraightSidewayLines() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      armsUp();

      // enable the use of body momentum in the controller
      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);

      // change the walking parameters
      for (RobotSide robotSide : RobotSide.values)
      {
         autoCropToLineAfterExploration.get(robotSide).set(true);
         holdFlatDuringExploration.get(robotSide).set(true);
         holdFlatDuringHoldPosition.get(robotSide).set(true);
         smartHoldPosition.get(robotSide).set(false);
         explorationMethods.get(robotSide).set(ExplorationMethod.FAST_LINE);
      }

      doFootExplorationInTransferToStanding.set(true);
      percentageChickenSupport.set(0.4);
      timeBeforeExploring.set(1.0);
      transferTime.set(0.15);
      swingTime.set(0.8);
      doToeOffIfPossible.set(false);

      for (int i = 0; i < 5; i++)
      {
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         ArrayList<Point2d> newContactPoints = generateContactPointsForRotatedLineOfContact(Math.PI/2.0, 0.0, 0.0);
         contactPointController.setNewContacts(newContactPoints, robotSide, true);

         FootstepDataListMessage message = new FootstepDataListMessage();
         FootstepDataMessage footstepData = new FootstepDataMessage();

         ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
         FramePoint placeToStepInWorld = new FramePoint(soleFrame, 0.0, 0.0, 0.0);
         placeToStepInWorld.changeFrame(worldFrame);
         placeToStepInWorld.setX(0.3 * (double)i);

         footstepData.setLocation(placeToStepInWorld.getPointCopy());
         footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
         footstepData.setRobotSide(robotSide);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         message.add(footstepData);

         drcSimulationTestHelper.send(message);
         boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 100.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 300000)
   public void testWalkingOnStraightForwardLines() throws SimulationExceededMaximumTimeException
   {
      setupTest();
      armsUp();

      // enable the use of body momentum in the controller
      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);

      // change the walking parameters
      for (RobotSide robotSide : RobotSide.values)
      {
         autoCropToLineAfterExploration.get(robotSide).set(true);
         holdFlatDuringExploration.get(robotSide).set(true);
         holdFlatDuringHoldPosition.get(robotSide).set(true);
         smartHoldPosition.get(robotSide).set(false);
         explorationMethods.get(robotSide).set(ExplorationMethod.FAST_LINE);
      }

      doFootExplorationInTransferToStanding.set(true);
      percentageChickenSupport.set(0.4);
      timeBeforeExploring.set(1.0);
      transferTime.set(0.15);
      swingTime.set(0.8);
      doToeOffIfPossible.set(false);

      for (int i = 0; i < 5; i++)
      {
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         ArrayList<Point2d> newContactPoints = generateContactPointsForRotatedLineOfContact(0.0, 0.0, 0.0);
         contactPointController.setNewContacts(newContactPoints, robotSide, true);

         FootstepDataListMessage message = new FootstepDataListMessage();
         FootstepDataMessage footstepData = new FootstepDataMessage();

         ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
         FramePoint placeToStepInWorld = new FramePoint(soleFrame, 0.0, 0.0, 0.0);
         placeToStepInWorld.changeFrame(worldFrame);
         placeToStepInWorld.setX(0.3 * (double)i);

         footstepData.setLocation(placeToStepInWorld.getPointCopy());
         footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
         footstepData.setRobotSide(robotSide);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         message.add(footstepData);

         drcSimulationTestHelper.send(message);
         boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);
         assertTrue(success);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 100.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 300000)
   public void testWalkingOnLines() throws SimulationExceededMaximumTimeException
   {
      Random random = new Random(49039845179L);

      setupTest();
      armsUp();

      // enable the use of body momentum in the controller
      allowUpperBodyMomentumInSingleSupport.set(true);
      allowUpperBodyMomentumInDoubleSupport.set(true);
      allowUsingHighMomentumWeight.set(true);

      // change the walking parameters
      for (RobotSide robotSide : RobotSide.values)
      {
         autoCropToLineAfterExploration.get(robotSide).set(true);
         holdFlatDuringExploration.get(robotSide).set(true);
         holdFlatDuringHoldPosition.get(robotSide).set(true);
         smartHoldPosition.get(robotSide).set(false);
         explorationMethods.get(robotSide).set(ExplorationMethod.FAST_LINE);
      }

      doFootExplorationInTransferToStanding.set(true);
      percentageChickenSupport.set(0.4);
      timeBeforeExploring.set(1.0);
      transferTime.set(0.15);
      swingTime.set(0.8);
      doToeOffIfPossible.set(false);

      for (int i = 0; i < 15; i++)
      {
         RobotSide robotSide = i%2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         ArrayList<Point2d> newContactPoints = generateContactPointsForRandomRotatedLineOfContact(random);
         contactPointController.setNewContacts(newContactPoints, robotSide, true);

         FootstepDataListMessage message = new FootstepDataListMessage();
         FootstepDataMessage footstepData = new FootstepDataMessage();

         ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(robotSide);
         FramePoint placeToStepInWorld = new FramePoint(soleFrame, 0.0, 0.0, 0.0);
         placeToStepInWorld.changeFrame(worldFrame);
         placeToStepInWorld.setX(0.3 * (double)i);

         footstepData.setLocation(placeToStepInWorld.getPointCopy());
         footstepData.setOrientation(new Quat4d(0.0, 0.0, 0.0, 1.0));
         footstepData.setRobotSide(robotSide);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         message.add(footstepData);

         drcSimulationTestHelper.send(message);
         boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);
         assertTrue(success);
      }
   }

   private ArrayList<Point2d> generateContactPointsForRandomRotatedLineOfContact(Random random)
   {
      double angle = RandomTools.generateRandomDouble(random, Math.PI);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double footLengthForLineOrigin = 0.5 * walkingControllerParameters.getFootLength();
      double footWidthForLineOrigin = 0.5 * walkingControllerParameters.getFootWidth();

      double x = RandomTools.generateRandomDouble(random, footLengthForLineOrigin) - footLengthForLineOrigin/2.0;
      double y = RandomTools.generateRandomDouble(random, footWidthForLineOrigin) - footWidthForLineOrigin/2.0;

      return generateContactPointsForRotatedLineOfContact(angle, x, y);
   }

   private ArrayList<Point2d> generateContactPointsForRotatedLineOfContact(double angle, double xLine, double yLine)
   {
      double lineWidth = 0.01;

      // build default foot polygon:
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double footForwardOffset = walkingControllerParameters.getFootForwardOffset();
      double footBackwardOffset = walkingControllerParameters.getFootBackwardOffset();
      double footWidth = walkingControllerParameters.getFootWidth();
      double toeWidth = walkingControllerParameters.getToeWidth();

      ArrayList<Point2d> soleVertices = new ArrayList<Point2d>();
      soleVertices.add(new Point2d(footForwardOffset, toeWidth / 2.0));
      soleVertices.add(new Point2d(footForwardOffset, -toeWidth / 2.0));
      soleVertices.add(new Point2d(-footBackwardOffset, -footWidth / 2.0));
      soleVertices.add(new Point2d(-footBackwardOffset, footWidth / 2.0));
      ConvexPolygon2d solePolygon = new ConvexPolygon2d(soleVertices);
      solePolygon.update();

      // shrink polygon and project line origin inside
      ConvexPolygon2d shrunkSolePolygon = new ConvexPolygon2d();
      ConvexPolygonShrinker shrinker = new ConvexPolygonShrinker();
      shrinker.shrinkConstantDistanceInto(solePolygon, lineWidth/2.0 + (footWidth-toeWidth)/2.0, shrunkSolePolygon);

      Point2d lineOrigin = new Point2d(xLine, yLine);
      shrunkSolePolygon.orthogonalProjection(lineOrigin);

      // transform line and compute intersections with default foot polygon
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(angle);
      transform.setTranslation(lineOrigin.getX(), lineOrigin.getY(), 0.0);

      Line2d line = new Line2d(new Point2d(0.0, 0.0), new Vector2d(1.0, 0.0));
      line.applyTransform(transform);

      line.shiftToLeft(lineWidth/2.0);
      Point2d[] leftIntersections = solePolygon.intersectionWith(line);
      line.shiftToRight(lineWidth);
      Point2d[] rightIntersections = solePolygon.intersectionWith(line);

      ArrayList<Point2d> ret = new ArrayList<Point2d>();
      ret.add(leftIntersections[0]);
      ret.add(leftIntersections[1]);
      ret.add(rightIntersections[0]);
      ret.add(rightIntersections[1]);
      return ret;
   }

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

   private void setupTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // create simulation test helper
      FlatGroundEnvironment emptyEnvironment = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();
      DRCStartingLocation startingLocation = new DRCStartingLocation()
      {
         @Override
         public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
         {
            return location;
         }
      };
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(emptyEnvironment, className, startingLocation, simulationTestingParameters, robotModel);

      // increase ankle damping to match the real robot better
      DoubleYoVariable damping_l_akx = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_akx");
      DoubleYoVariable damping_l_aky = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("b_damp_l_leg_aky");
      DoubleYoVariable damping_r_akx = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_akx");
      DoubleYoVariable damping_r_aky = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("b_damp_r_leg_aky");
      damping_l_akx.set(1.0);
      damping_l_aky.set(1.0);
      damping_r_akx.set(1.0);
      damping_r_aky.set(1.0);

      // get foot states
      for (RobotSide robotSide : RobotSide.values)
      {
         String variableName = robotSide.getCamelCaseNameForStartOfExpression() + "FootState";
         EnumYoVariable<ConstraintType> footState = (EnumYoVariable<ConstraintType>) drcSimulationTestHelper.getYoVariable(variableName);
         footStates.put(robotSide, footState);
      }

      // get a bunch of relevant variables
      doFootExplorationInTransferToStanding = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doFootExplorationInTransferToStanding");
      transferTime = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("transferTime");
      swingTime = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("swingTime");
      percentageChickenSupport = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("PercentageChickenSupport");
      timeBeforeExploring = (DoubleYoVariable) drcSimulationTestHelper.getYoVariable("ExplorationState_TimeBeforeExploring");
      for (RobotSide robotSide : RobotSide.values)
      {
         String footName = drcSimulationTestHelper.getControllerFullRobotModel().getFoot(robotSide).getName();
         String footControlNamespace = robotSide.getLowerCaseName() + "FootControlModule";

         BooleanYoVariable autoCrop = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable(footName + "AutoCropToLineAfterExploration");
         autoCropToLineAfterExploration.put(robotSide, autoCrop);
         BooleanYoVariable holdFlatDuringExploration = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable(footControlNamespace, footName + "DoHoldFootFlatOrientation");
         this.holdFlatDuringExploration.put(robotSide, holdFlatDuringExploration);
         BooleanYoVariable holdFlatDuringHoldPosition = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("ExploreFootPolygon", footName + "DoHoldFootFlatOrientation");
         this.holdFlatDuringHoldPosition.put(robotSide, holdFlatDuringHoldPosition);
         BooleanYoVariable smartHoldPosition = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable(footControlNamespace, footName + "DoSmartHoldPosition");
         this.smartHoldPosition.put(robotSide, smartHoldPosition);
         EnumYoVariable<ExplorationMethod> explorationMethod = (EnumYoVariable<ExplorationMethod>) drcSimulationTestHelper.getYoVariable(footName + "ExplorationMethod");
         explorationMethods.put(robotSide, explorationMethod);
      }
      allowUpperBodyMomentumInSingleSupport = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInSingleSupport");
      allowUpperBodyMomentumInDoubleSupport = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUpperBodyMomentumInDoubleSupport");
      allowUsingHighMomentumWeight = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("allowUsingHighMomentumWeight");
      doToeOffIfPossible = (BooleanYoVariable) drcSimulationTestHelper.getYoVariable("doToeOffIfPossible");

      // setup camera
      Point3d cameraFix = new Point3d(0.0, 0.0, 1.0);
      Point3d cameraPosition = new Point3d(-10.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      contactPointController = new ContactPointController();
      drcSimulationTestHelper.addRobotControllerOnControllerThread(contactPointController);
      setupSupportViz();

      ThreadTools.sleep(1000);
   }

   private static final double[] rightHandStraightSideJointAngles = new double[] {-0.5067668142160446, -0.3659876546358431, 1.7973796317575155, -1.2398714600960365, -0.005510224629709242, 0.6123343067479899, 0.12524505635696856};
   private static final double[] leftHandStraightSideJointAngles = new double[] {0.61130147334225, 0.22680071472282162, 1.6270339908033258, 1.2703560974484844, 0.10340544060719102, -0.6738299572358809, 0.13264785356924128};
   private static final SideDependentList<double[]> straightArmConfigs = new SideDependentList<>();
   static
   {
      straightArmConfigs.put(RobotSide.LEFT, leftHandStraightSideJointAngles);
      straightArmConfigs.put(RobotSide.RIGHT, rightHandStraightSideJointAngles);
   }

   private void armsUp() throws SimulationExceededMaximumTimeException
   {
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      // bring the arms in a stretched position
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
         armTrajectoryMessage.robotSide = robotSide;
         double[] armConfig = straightArmConfigs.get(robotSide);
         armTrajectoryMessage.jointTrajectoryMessages = new OneDoFJointTrajectoryMessage[armConfig.length];
         for (int i = 0; i < armConfig.length; i++)
         {
            TrajectoryPoint1DMessage trajectoryPoint = new TrajectoryPoint1DMessage();
            trajectoryPoint.position = armConfig[i];
            trajectoryPoint.time = 1.0;
            OneDoFJointTrajectoryMessage jointTrajectory = new OneDoFJointTrajectoryMessage();
            jointTrajectory.trajectoryPoints = new TrajectoryPoint1DMessage[] {trajectoryPoint};
            armTrajectoryMessage.jointTrajectoryMessages[i] = jointTrajectory;
         }
         drcSimulationTestHelper.send(armTrajectoryMessage);
      }

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1);
   }

   private void setupSupportViz()
   {
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      supportPolygons = new SideDependentList<YoFrameConvexPolygon2d>();
      supportPolygons.set(RobotSide.LEFT, new YoFrameConvexPolygon2d("FootPolygonLeft", "", worldFrame, 4, registry));
      supportPolygons.set(RobotSide.RIGHT, new YoFrameConvexPolygon2d("FootPolygonRight", "", worldFrame, 4, registry));

      footContactsInAnkleFrame = new SideDependentList<ArrayList<Point2d>>();
      footContactsInAnkleFrame.set(RobotSide.LEFT, null);
      footContactsInAnkleFrame.set(RobotSide.RIGHT, null);

      yoGraphicsListRegistry.registerArtifact("SupportLeft", new YoArtifactPolygon("SupportLeft", supportPolygons.get(RobotSide.LEFT), Color.BLACK, false));
      yoGraphicsListRegistry.registerArtifact("SupportRight", new YoArtifactPolygon("SupportRight", supportPolygons.get(RobotSide.RIGHT), Color.BLACK, false));

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      drcSimulationTestHelper.addRobotControllerOnControllerThread(new VizUpdater());
   }

   private class VizUpdater implements RobotController
   {
      FrameConvexPolygon2d footSupport = new FrameConvexPolygon2d(worldFrame);
      FramePoint2d point = new FramePoint2d(worldFrame);
      FramePoint point3d = new FramePoint();

      @Override
      public void doControl()
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            ArrayList<Point2d> contactPoints = footContactsInAnkleFrame.get(robotSide);
            if (contactPoints == null) continue;

            footSupport.clear(worldFrame);
            ReferenceFrame ankleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getEndEffectorFrame(robotSide, LimbName.LEG);
            ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrames().get(robotSide);

            for (int i = 0; i < contactPoints.size(); i++)
            {
               point3d.setXYIncludingFrame(ankleFrame, contactPoints.get(i));
               point3d.changeFrame(soleFrame);
               point3d.setZ(0.0);
               point3d.changeFrame(worldFrame);
               point3d.getFramePoint2d(point);
               footSupport.addVertex(point.getPoint());
            }

            footSupport.update();
            supportPolygons.get(robotSide).setFrameConvexPolygon2d(footSupport);
         }
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return null;
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
   };

   private class ContactPointController implements RobotController
   {
      private ArrayList<Point2d> newContactPoints = null;
      private RobotSide robotSide = null;

      private AtomicBoolean setNewContactPoints = new AtomicBoolean(false);
      private boolean setOnStep = false;

      /**
       * Changes the foot contact points of the robot.
       * The contact points can be changed immediately or when the foot is in swing.
       *
       * @param newContactPoints
       * @param robotSide
       * @param setOnStep
       */
      public void setNewContacts(ArrayList<Point2d> newContactPoints, RobotSide robotSide, boolean setOnStep)
      {
         if (setNewContactPoints.get())
         {
            System.err.println("New contact points are already waiting to be set.");
            return;
         }

         this.newContactPoints = newContactPoints;
         this.robotSide = robotSide;
         this.setOnStep = setOnStep;

         setNewContactPoints.set(true);
      }

      @Override
      public void initialize()
      {
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return null;
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

      @Override
      public void doControl()
      {
         if (setNewContactPoints.get())
         {
            if (!setOnStep)
               setNewContacts();
            else if (footStates.get(robotSide).getEnumValue() == ConstraintType.SWING)
               setNewContacts();
         }
      }

      private void setNewContacts()
      {
         String footJointName = drcSimulationTestHelper.getControllerFullRobotModel().getFoot(robotSide).getParentJoint().getName();
         HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

         int pointIndex = 0;
         ArrayList<GroundContactPoint> allGroundContactPoints = robot.getAllGroundContactPoints();

         for (GroundContactPoint point : allGroundContactPoints)
         {
            Joint parentJoint = point.getParentJoint();

            if (parentJoint.getName().equals(footJointName))
            {
               Point2d newContactPoint = newContactPoints.get(pointIndex);

               point.setIsInContact(false);
               Vector3d offset = new Vector3d();
               point.getOffset(offset);

               offset.setX(newContactPoint.getX());
               offset.setY(newContactPoint.getY());

               point.setOffsetJoint(offset);
               pointIndex++;
            }
         }

         if (footContactsInAnkleFrame != null)
         {
            footContactsInAnkleFrame.set(robotSide, newContactPoints);
         }

         setNewContactPoints.set(false);
      }

   }
}
