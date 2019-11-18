package us.ihmc.valkyrie.torquespeedcurve;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static us.ihmc.avatar.testTools.EndToEndTestTools.computeWalkingDuration;
import static us.ihmc.robotics.Assert.assertTrue;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.dataExporter.DataExporterExcelWorkbookCreator;
import us.ihmc.simulationConstructionSetTools.dataExporter.TorqueSpeedDataExporterGraphCreator;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.StaircaseEnvironment;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.valkyrie.ValkyrieInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieJointMap;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class ValkyrieTorqueSpeedCurveEndToEndTest
{
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private ValkyrieJointMap jointMap = new ValkyrieJointMap();

   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

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

   @Test
   public void testStepUpInchWithSquareUpReal() throws Exception
   {
      for (double stepHeight : Arrays.asList(9.5, 6.0, 7.25, 8.25))
      {
         testStepUpWithSquareUp(stepHeight, getRealRobotWalkingParameters(), getDataOuputFolderForRealRobotParams());
      }
   }

   @Test
   public void testStepUpInchWithSquareUpSim() throws Exception
   {
      for (double stepHeight : Arrays.asList(9.5, 6.0, 7.25, 8.25))
      {
         testStepUpWithSquareUp(stepHeight, getSCSWalkingParameters(), getDataOuputFolderForSimParams());
      }
   }

   public void testStepUpWithSquareUp(double stepHeightInches, WalkingControllerParameters walkingControllerParameters, File dataOutputFolder)
         throws SimulationExceededMaximumTimeException
   {
      showMemoryUsageBeforeTest();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double stepStart = 1.0;
      double stepHeight = 2.54 / 100.0 * stepHeightInches;
      StepUpEnvironment stepUp = new StepUpEnvironment(stepStart, stepHeight);

      DRCRobotModel robotModel = getRobotModel(false);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, stepUp);
      drcSimulationTestHelper.createSimulation("StepUpWithSquareUpFast");
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setCameraFix(1.0, 0.0, 0.8);
      scs.setCameraPosition(1.0, -8.0, 1.0);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      double xGoal = 2.0;

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      double stepLength = steppingParameters.getDefaultStepLength();
      double stepWidth = steppingParameters.getInPlaceWidth();

      double footLength = steppingParameters.getFootLength();
      HeightMapWithNormals heightMap = stepUp.getTerrainObject3D().getHeightMapIfAvailable();

      FootstepDataListMessage footsteps;
      footsteps = stepTo(0.0, stepStart - 0.5 * footLength - 0.05, stepLength, stepWidth, RobotSide.LEFT, heightMap, null, false, true);
      setTimings(footsteps, walkingControllerParameters);
      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(computeWalkingDuration(footsteps, walkingControllerParameters) + 0.25));
      scs.setInPoint();

      footsteps = stepTo(stepStart + 0.5 * footLength + 0.05, xGoal, stepLength, stepWidth, RobotSide.LEFT, heightMap, null, true, true);
      setTimings(footsteps, walkingControllerParameters);
      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(computeWalkingDuration(footsteps, walkingControllerParameters) + 0.25));

      Point3D center = new Point3D(1.9552, 0.0, 1.3);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.1);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      String dataNameSuffix = "StepUpWithSquareUp" + stepHeightInches;
      String info = "Square up -> step up a " + stepHeightInches + " inches high step -> square up.";
      exportTorqueSpeedCurves(scs, dataOutputFolder, dataNameSuffix, info);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      destroySimulationAndRecycleMemory();
   }

   @Test
   public void testStepUpInchWithoutSquareUpReal() throws Exception
   {
      for (double stepHeight : Arrays.asList(9.5, 6.0, 7.25, 8.25))
      {
         testStepUpWithoutSquareUp(stepHeight, getRealRobotWalkingParameters(), getDataOuputFolderForRealRobotParams());
      }
   }

   @Test
   public void testStepUpInchWithoutSquareUpSim() throws Exception
   {
      for (double stepHeight : Arrays.asList(9.5, 6.0, 7.25, 8.25))
      {
         testStepUpWithoutSquareUp(stepHeight, getSCSWalkingParameters(), getDataOuputFolderForSimParams());
      }
   }

   public void testStepUpWithoutSquareUp(double stepHeightInches, WalkingControllerParameters walkingControllerParameters, File dataOutputFolder)
         throws SimulationExceededMaximumTimeException
   {
      showMemoryUsageBeforeTest();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double stepStart = 1.0;
      double stepHeight = 2.54 / 100.0 * stepHeightInches;
      StepUpEnvironment stepUp = new StepUpEnvironment(stepStart, stepHeight);

      DRCRobotModel robotModel = getRobotModel(false);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, stepUp);
      drcSimulationTestHelper.createSimulation("StepUpWithoutSquareUp");
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setCameraFix(1.0, 0.0, 0.8);
      scs.setCameraPosition(1.0, -8.0, 1.0);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      scs.setInPoint();

      double xGoal = 2.0;
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      double stepLength = steppingParameters.getDefaultStepLength();
      double stepWidth = steppingParameters.getInPlaceWidth();

      double footLength = steppingParameters.getFootLength();
      HeightMapWithNormals heightMap = stepUp.getTerrainObject3D().getHeightMapIfAvailable();

      stepTo(0.0, stepStart - 0.5 * footLength - 0.15, stepLength, stepWidth, RobotSide.LEFT, heightMap, footsteps, false, false);
      RobotSide firstSide = RobotSide.fromByte(footsteps.getFootstepDataList().getLast().getRobotSide()).getOppositeSide();
      stepTo(stepStart + 0.5 * footLength + 0.05, xGoal, stepLength, stepWidth, firstSide, heightMap, footsteps, false, true);
      setTimings(footsteps, walkingControllerParameters);

      drcSimulationTestHelper.publishToController(footsteps);

      double walkingDuration = EndToEndTestTools.computeWalkingDuration(footsteps, walkingControllerParameters);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingDuration + 0.25);
      assertTrue(success);

      Point3D center = new Point3D(1.9552, 0.0, 1.3);
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.1);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      String dataNameSuffix = "StepUpWithoutSquareUp" + stepHeightInches;
      String info = "Step up a " + stepHeightInches + " inches high step while walking";
      exportTorqueSpeedCurves(scs, dataOutputFolder, dataNameSuffix, info);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      destroySimulationAndRecycleMemory();
   }

   @Test
   public void testWalkDownSlope45DegReal() throws Exception
   {
      testWalkSlope(Math.toRadians(45.0),
                    0.5 * getRealRobotSteppingParameters().getDefaultStepLength(),
                    getRealRobotWalkingParameters(),
                    getDataOuputFolderForRealRobotParams());
   }

   @Test
   public void testWalkDownSlope45DegSim() throws Exception
   {
      testWalkSlope(Math.toRadians(45.0),
                    0.33 * getSCSSteppingParameters().getDefaultStepLength(),
                    getSCSWalkingParameters(),
                    getDataOuputFolderForSimParams());
   }

   @Test
   public void testWalkDownSlope30DegReal() throws Exception
   {
      testWalkSlope(Math.toRadians(30.0),
                    0.5 * getRealRobotSteppingParameters().getDefaultStepLength(),
                    getRealRobotWalkingParameters(),
                    getDataOuputFolderForRealRobotParams());
   }

   @Test
   public void testWalkDownSlope30DegSim() throws Exception
   {
      testWalkSlope(Math.toRadians(30.0), 0.5 * getSCSSteppingParameters().getDefaultStepLength(), getSCSWalkingParameters(), getDataOuputFolderForSimParams());
   }

   @Disabled // TODO This exhibits a few control issues and doesn't look realistic at all
   @Test
   public void testWalkUpSlope45DegReal() throws Exception
   {
      testWalkSlope(Math.toRadians(-45.0),
                    0.5 * getRealRobotSteppingParameters().getDefaultStepLength(),
                    getRealRobotWalkingParameters(),
                    getDataOuputFolderForRealRobotParams());
   }

   @Disabled // TODO This exhibits a few control issues and doesn't look realistic at all
   @Test
   public void testWalkUpSlope45DegSim() throws Exception
   {
      testWalkSlope(Math.toRadians(-45.0),
                    0.33 * getSCSSteppingParameters().getDefaultStepLength(),
                    getSCSWalkingParameters(),
                    getDataOuputFolderForSimParams());
   }

   @Test
   public void testWalkUpSlope30DegReal() throws Exception
   {
      testWalkSlope(Math.toRadians(-30.0),
                    0.5 * getRealRobotSteppingParameters().getDefaultStepLength(),
                    getRealRobotWalkingParameters(),
                    getDataOuputFolderForRealRobotParams());
   }

   @Test
   public void testWalkUpSlope30DegSim() throws Exception
   {
      testWalkSlope(Math.toRadians(-30.0),
                    0.5 * getSCSSteppingParameters().getDefaultStepLength(),
                    getSCSWalkingParameters(),
                    getDataOuputFolderForSimParams());
   }

   public void testWalkSlope(double slopeAngle, double stepLength, WalkingControllerParameters walkingControllerParameters, File dataOutputFolder)
         throws SimulationExceededMaximumTimeException
   {
      showMemoryUsageBeforeTest();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      SlopeEnvironment slope = new SlopeEnvironment(slopeAngle);

      DRCRobotModel robotModel = getRobotModel(slopeAngle < 0.0);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, slope);
      drcSimulationTestHelper.setInitialSetup(initialSetupForSlope(slopeAngle, slope.getTerrainObject3D().getHeightMapIfAvailable()));
      drcSimulationTestHelper.createSimulation("StepUpWithoutSquareUp");
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      double cameraX = 2.0;
      double cameraZ = 0.8 + slope.getTerrainObject3D().getHeightMapIfAvailable().heightAt(cameraX, 0.0, 0.0);
      scs.setCameraFix(cameraX, 0.0, cameraZ);
      scs.setCameraPosition(cameraX, -8.0, cameraZ);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      scs.setInPoint();

      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      double stepWidth = steppingParameters.getInPlaceWidth();

      HeightMapWithNormals heightMap = slope.getTerrainObject3D().getHeightMapIfAvailable();

      stepTo(0.2, 5.0, stepLength, stepWidth, RobotSide.LEFT, heightMap, footsteps, true, true);
      setTimings(footsteps, walkingControllerParameters);
      footsteps.getFootstepDataList().forEach(footstep -> footstep.getOrientation().setToPitchOrientation(slopeAngle));
      footsteps.getFootstepDataList().forEach(footstep -> footstep.setTrajectoryType(TrajectoryType.CUSTOM.toByte()));
      computeDefaultWaypointPrositions(footsteps, new double[] {0.15, 0.75}, 0.075);
      footsteps.setOffsetFootstepsHeightWithExecutionError(true);

      drcSimulationTestHelper.publishToController(footsteps);

      double walkingDuration = EndToEndTestTools.computeWalkingDuration(footsteps, walkingControllerParameters);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingDuration + 0.25);
      assertTrue(success);

      //      Point3D center = new Point3D(1.9552, 0.0, 1.3);
      //      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.1);
      //      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      //      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      String dataNameSuffix = "Walk" + (slopeAngle < 0.0 ? "Up" : "Down") + "Slope" + Integer.toString((int) Math.toDegrees(slopeAngle)) + "Deg";
      String info = "Walking " + (slopeAngle < 0.0 ? "up" : "down") + " slope of " + Integer.toString((int) Math.toDegrees(slopeAngle)) + " degrees";
      exportTorqueSpeedCurves(scs, dataOutputFolder, dataNameSuffix, info);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      destroySimulationAndRecycleMemory();
   }

   @Test
   public void testUpstairsReal() throws Exception
   {
      testUpstairs(7.0, 3, getRealRobotWalkingParameters(), getDataOuputFolderForRealRobotParams());
      testUpstairs(9.5, 10, getRealRobotWalkingParameters(), getDataOuputFolderForRealRobotParams());
   }

   @Test
   public void testUpstairsSim() throws Exception
   {
      testUpstairs(7.0, 3, getSCSWalkingParameters(), getDataOuputFolderForSimParams());
      /*
       * TODO Needs: SwingTrajectoryParameters.useSingularityAvoidanceInSupport() to be false and disable
       * smoothing in the CenterOfMassHeightControlState.computeDesiredCoMHeightAcceleration(...)
       */
      assertFalse(getSCSWalkingParameters().getSwingTrajectoryParameters().useSingularityAvoidanceInSupport());
      testUpstairs(9.5, 10, getSCSWalkingParameters(), getDataOuputFolderForSimParams());
   }

   public void testUpstairs(double stairStepHeightInches, int numberOfStairSteps, WalkingControllerParameters walkingControllerParameters,
                            File dataOutputFolder)
         throws SimulationExceededMaximumTimeException
   {
      showMemoryUsageBeforeTest();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double stairStepHeight = 2.54 / 100.0 * stairStepHeightInches;
      double stairStepLength = 1.5 * walkingControllerParameters.getSteppingParameters().getActualFootLength();
      StaircaseEnvironment staircase = new StaircaseEnvironment(numberOfStairSteps, stairStepHeight, stairStepLength);

      DRCRobotModel robotModel = getRobotModel(false);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, staircase);
      drcSimulationTestHelper.createSimulation("StepUpWithoutSquareUp");
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      double xGoal = 0.6 + numberOfStairSteps * stairStepLength;
      double cameraX = xGoal / 2.0;
      double cameraZ = 0.8 + staircase.getCombinedTerrainObject3D().getHeightMapIfAvailable().heightAt(cameraX, 0.0, 10.0);
      scs.setCameraFix(cameraX, 0.0, cameraZ);
      scs.setCameraPosition(cameraX, -8.0, cameraZ);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FramePoint3D pelvisPosition = new FramePoint3D(drcSimulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      double desiredHeight = pelvisPosition.getZ() + 0.01;
      PelvisHeightTrajectoryMessage pelvisHeightMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.2, desiredHeight);
      drcSimulationTestHelper.publishToController(pelvisHeightMessage);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);
      scs.setInPoint();

      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      double stepLength = steppingParameters.getDefaultStepLength();
      double stepWidth = steppingParameters.getInPlaceWidth();

      HeightMapWithNormals heightMap = staircase.getTerrainObject3D().getHeightMapIfAvailable();

      double firstStep = 0.6;
      stepTo(0.0, firstStep - 0.5 * stairStepLength, stepLength, stepWidth, RobotSide.LEFT, heightMap, footsteps, false, false);
      RobotSide firstSide = RobotSide.fromByte(footsteps.getFootstepDataList().getLast().getRobotSide()).getOppositeSide();
      stepTo(firstStep + 0.5 * stairStepLength, xGoal, stairStepLength, stepWidth, firstSide, heightMap, footsteps, false, true);
      setTimings(footsteps, walkingControllerParameters);

      drcSimulationTestHelper.publishToController(footsteps);

      double walkingDuration = EndToEndTestTools.computeWalkingDuration(footsteps, walkingControllerParameters);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingDuration + 1.0);
      assertTrue(success);

      Point3D center = new Point3D(xGoal, 0.0, 1.1 + staircase.getCombinedTerrainObject3D().getHeightMapIfAvailable().heightAt(xGoal, 0.0, 10.0));
      Vector3D plusMinusVector = new Vector3D(0.1, 0.1, 0.1);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      String dataNameSuffix = "Upstairs" + stairStepHeightInches + "StepHeight";
      String info = "Walking upstairs with " + stairStepHeightInches + " inches high steps";
      exportTorqueSpeedCurves(scs, dataOutputFolder, dataNameSuffix, info);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      destroySimulationAndRecycleMemory();
   }

   private static ValkyrieInitialSetup initialSetupForSlope(double slopeAngle, HeightMap heightMap)
   {
      return new ValkyrieInitialSetup(0.0, 0.0)
      {
         @Override
         protected void setActuatorPositions(FloatingRootJointRobot robot, DRCRobotJointMap jointMap)
         {
            super.setActuatorPositions(robot, jointMap);

            for (RobotSide robotSide : RobotSide.values)
            {
               String hipPitch = jointMap.getLegJointName(robotSide, LegJointName.HIP_PITCH);
               String anklePitch = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_PITCH);

               OneDegreeOfFreedomJoint anklePitchJoint = robot.getOneDegreeOfFreedomJoint(anklePitch);
               OneDegreeOfFreedomJoint hipPitchJoint = robot.getOneDegreeOfFreedomJoint(hipPitch);
               hipPitchJoint.setQ(hipPitchJoint.getQ() - 0.05);
               anklePitchJoint.setQ(anklePitchJoint.getQ() + slopeAngle - 0.05);
            }
            robot.update();
         }

         @Override
         protected void positionRobotInWorld(HumanoidFloatingRootJointRobot robot)
         {
            super.positionRobotInWorld(robot);

            for (RobotSide robotSide : RobotSide.values)
            {
               List<GroundContactPoint> footGCs = robot.getFootGroundContactPoints(robotSide);

               for (GroundContactPoint gc : footGCs)
               {
                  double heightAt = heightMap.heightAt(gc.getX(), gc.getY(), 0.0);
                  robot.getRootJoint().getQz().add(heightAt - gc.getZ());
                  robot.update();
               }
            }
         }
      };
   }

   private static void computeDefaultWaypointPrositions(FootstepDataListMessage footsteps, double[] percentages, double swingHeight)
   {
      Map<RobotSide, List<FootstepDataMessage>> stepsBySide = footsteps.getFootstepDataList().stream()
                                                                       .collect(Collectors.groupingBy(step -> RobotSide.fromByte(step.getRobotSide())));

      for (RobotSide robotSide : RobotSide.values)
      {
         List<FootstepDataMessage> steps = stepsBySide.get(robotSide);

         for (int i = 1; i < steps.size(); i++)
         {
            for (double percentage : percentages)
            {
               Point3D waypoint = computeOneSwingWaypoint(steps.get(i - 1).getLocation(), steps.get(i).getLocation(), percentage, swingHeight);
               steps.get(i).getCustomPositionWaypoints().add().set(waypoint);
            }
         }
      }
   }

   private static Point3D computeOneSwingWaypoint(Point3DReadOnly start, Point3DReadOnly end, double percentage, double swingHeight)
   {
      Point3D waypoint = new Point3D();
      waypoint.interpolate(start, end, percentage);

      Vector3D stepDirection = new Vector3D();
      Vector3D perpendicularDirection = new Vector3D();
      stepDirection.sub(end, start);
      perpendicularDirection.cross(Axis.Z, stepDirection);
      perpendicularDirection.normalize();
      Vector3D swingHeightSupportVector = new Vector3D();
      swingHeightSupportVector.cross(stepDirection, perpendicularDirection);
      swingHeightSupportVector.normalize();

      waypoint.scaleAdd(swingHeight, swingHeightSupportVector, waypoint);

      return waypoint;
   }

   private static void setTimings(FootstepDataListMessage footsteps, WalkingControllerParameters parametersWithTimings)
   {
      footsteps.setDefaultSwingDuration(parametersWithTimings.getDefaultSwingTime());
      footsteps.setDefaultTransferDuration(parametersWithTimings.getDefaultTransferTime());
   }

   private static FootstepDataListMessage stepTo(double xStart, double xGoal, double stepLength, double stepWidth, RobotSide firstStepSide, HeightMap heightMap,
                                                 FootstepDataListMessage stepsToPack, boolean squareUpStart, boolean squareUpEnd)
   {
      if (stepsToPack == null)
         stepsToPack = new FootstepDataListMessage();

      double x = xStart;
      RobotSide stepSide = firstStepSide;

      if (squareUpStart)
      {
         setupFootstep(stepSide, x, stepWidth, heightMap, stepsToPack.getFootstepDataList().add());
         stepSide = stepSide.getOppositeSide();
      }

      while (x < xGoal)
      {
         setupFootstep(stepSide, x, stepWidth, heightMap, stepsToPack.getFootstepDataList().add());

         x += stepLength;
         stepSide = stepSide.getOppositeSide();
      }

      setupFootstep(stepSide, xGoal, stepWidth, heightMap, stepsToPack.getFootstepDataList().add());
      if (squareUpEnd)
         setupFootstep(stepSide.getOppositeSide(), xGoal, stepWidth, heightMap, stepsToPack.getFootstepDataList().add());

      return stepsToPack;
   }

   private static void setupFootstep(RobotSide robotSide, double x, double stepWidth, HeightMap heightMap, FootstepDataMessage stepToPack)
   {
      stepToPack.setRobotSide(robotSide.toByte());
      setFootstepLocation(x, robotSide.negateIfRightSide(0.5 * stepWidth), heightMap, stepToPack);
   }

   private static void setFootstepLocation(double x, double y, HeightMap heightMap, FootstepDataMessage stepToPack)
   {
      stepToPack.getLocation().set(x, y, heightMap.heightAt(x, y, 10.0));
   }

   private ValkyrieRobotModel getRobotModel(boolean disableAnkleLimits)
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false)
      {
         @Override
         public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
         {
            if (disableAnkleLimits && jointHolder.getName().contains("AnklePitch"))
            {
               jointHolder.setLimits(-Math.PI, Math.PI);
            }
         }
      };
   }

   // Pattern-matched from TorqueSpeedDataExporter
   private static void exportTorqueSpeedCurves(SimulationConstructionSet scs, File dataParentFolder, String dataNameSuffix, String info)
   {
      Robot robot = scs.getRobots()[0];
      TorqueSpeedDataExporterGraphCreator graphCreator = new TorqueSpeedDataExporterGraphCreator(robot, scs.getDataBuffer());
      DataExporterExcelWorkbookCreator excelWorkbookCreator = new DataExporterExcelWorkbookCreator(robot, scs.getDataBuffer());

      // Stop the sim and disable the GUI:
      scs.stop();
      scs.disableGUIComponents();

      // Wait till done running:
      while (scs.isSimulating())
      {
         ThreadTools.sleep(1000);
      }

      // Crop the Buffer to In/Out. This is important because of how we use the DataBuffer later and we assume that in point is at index=0:
      scs.cropBuffer();
      scs.gotoInPointNow();

      String timeStamp = FormattingTools.getDateString() + "_" + FormattingTools.getTimeString();
      String tagName = timeStamp + "_" + robot.getName() + "_" + dataNameSuffix;

      File dataFolder = new File(dataParentFolder, tagName);
      dataFolder.mkdir();

      System.out.println("Saving ReadMe");
      writeReadme(new File(dataFolder, tagName + ".txt"), info);
      System.out.println("Done Saving ReadMe");

      System.out.println("Saving data");
      scs.writeData(new File(dataFolder, tagName + ".data.gz"));
      System.out.println("Done Saving Data");

      System.out.println("Saving data in Matlab format");
      try
      {
         scs.writeMatlabData("all", new File(dataFolder, tagName + ".m"));
         System.out.println("Done Saving Data in Matlab format");
      }
      catch (OutOfMemoryError exception)
      {
         System.err.println("Ran out of memory while saving to Matlab format. Try again with fewer points.");
         exception.printStackTrace();
      }

      System.out.println("creating torque and speed spreadsheet");
      excelWorkbookCreator.createAndSaveTorqueAndSpeedSpreadSheet(dataFolder, tagName);
      System.out.println("done creating torque and speed spreadsheet");

      System.out.println("creating torque and speed graphs");
      // make graph directory inside destination directory
      File graphDirectory = new File(dataFolder, "graphs");
      graphDirectory.mkdir();
      graphCreator.createJointTorqueSpeedGraphs(graphDirectory, tagName, true, false);
      System.out.println("done creating torque and speed graphs");

      System.out.println("creating video");
      scs.getStandardSimulationGUI().getViewportPanel().getStandardGUIActions().createVideo(new File(dataFolder, tagName + "_Video.mov"));
      System.out.println("done creating video");

      scs.enableGUIComponents();
   }

   private static void writeReadme(File readmeFile, String info)
   {
      try
      {
         FileWriter out = new FileWriter(readmeFile);
         out.write(info);
         out.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private ValkyrieWalkingControllerParameters getRealRobotWalkingParameters()
   {
      return new ValkyrieWalkingControllerParameters(jointMap, RobotTarget.REAL_ROBOT);
   }

   private ValkyrieWalkingControllerParameters getSCSWalkingParameters()
   {
      return new ValkyrieWalkingControllerParameters(jointMap, RobotTarget.SCS);
   }

   private SteppingParameters getRealRobotSteppingParameters()
   {
      return getRealRobotWalkingParameters().getSteppingParameters();
   }

   private SteppingParameters getSCSSteppingParameters()
   {
      return getSCSWalkingParameters().getSteppingParameters();
   }

   private static File getDataOuputFolderForSimParams() throws IOException
   {
      Path path = Paths.get("D:/DataAndVideos/Valkyrie/SimParams");
      FileTools.ensureDirectoryExists(path);
      return path.toFile();
   }

   private static File getDataOuputFolderForRealRobotParams() throws IOException
   {
      Path path = Paths.get("D:/DataAndVideos/Valkyrie/RealRobotParams");
      FileTools.ensureDirectoryExists(path);
      return path.toFile();
   }
}
