package us.ihmc.valkyrie.torquespeedcurve;

import static us.ihmc.avatar.testTools.EndToEndTestTools.computeWalkingDuration;
import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyrieTorqueSpeedCurveEndToEndTest
{
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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

   @Test
   public void testStepUpWithSquareUpReal() throws SimulationExceededMaximumTimeException
   {
      testStepUpWithSquareUp(new ValkyrieWalkingControllerParameters(getRobotModel().getJointMap(), RobotTarget.REAL_ROBOT));
   }

   @Test
   public void testStepUpWithSquareUpSim() throws SimulationExceededMaximumTimeException
   {
      testStepUpWithSquareUp(new ValkyrieWalkingControllerParameters(getRobotModel().getJointMap(), RobotTarget.SCS));
   }

   public void testStepUpWithSquareUp(WalkingControllerParameters walkingControllerParameters) throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double stepStart = 1.0;
      double stepHeight = 2.54 / 100.0 * 9.5;
      StepUpEnvironment stepUp = new StepUpEnvironment(stepStart, stepHeight);

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, stepUp);
      drcSimulationTestHelper.createSimulation("StepUpWithSquareUpFast");
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setCameraFix(1.0, 0.0, 0.8);
      scs.setCameraPosition(1.0, -8.0, 1.0);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      double xGoal = 2.0;

      double x = 0.0;
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      double stepLength = steppingParameters.getDefaultStepLength();
      double stepWidth = steppingParameters.getInPlaceWidth();

      double footLength = steppingParameters.getFootLength();
      HeightMapWithNormals heightMap = stepUp.getTerrainObject3D().getHeightMapIfAvailable();

      FootstepDataListMessage footsteps;
      footsteps = stepTo(x, stepStart - 0.5 * footLength - 0.05, stepLength, stepWidth, RobotSide.LEFT, heightMap, null, false, true);
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

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testStepUpWithoutSquareUpReal() throws SimulationExceededMaximumTimeException
   {
      testStepUpWithoutSquareUp(new ValkyrieWalkingControllerParameters(getRobotModel().getJointMap(), RobotTarget.REAL_ROBOT));
   }

   @Test
   public void testStepUpWithoutSquareUpSim() throws SimulationExceededMaximumTimeException
   {
      testStepUpWithoutSquareUp(new ValkyrieWalkingControllerParameters(getRobotModel().getJointMap(), RobotTarget.SCS));
   }

   public void testStepUpWithoutSquareUp(WalkingControllerParameters walkingControllerParameters) throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double stepStart = 1.0;
      double stepHeight = 2.54 / 100.0 * 9.5;
      StepUpEnvironment stepUp = new StepUpEnvironment(stepStart, stepHeight);

      DRCRobotModel robotModel = getRobotModel();
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

      double x = 0.0;

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      double stepLength = steppingParameters.getDefaultStepLength();
      double stepWidth = steppingParameters.getInPlaceWidth();

      double footLength = steppingParameters.getFootLength();
      HeightMapWithNormals heightMap = stepUp.getTerrainObject3D().getHeightMapIfAvailable();

      stepTo(x, stepStart - 0.5 * footLength - 0.15, stepLength, stepWidth, RobotSide.LEFT, heightMap, footsteps, false, false);
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

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
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

   private ValkyrieRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

}
