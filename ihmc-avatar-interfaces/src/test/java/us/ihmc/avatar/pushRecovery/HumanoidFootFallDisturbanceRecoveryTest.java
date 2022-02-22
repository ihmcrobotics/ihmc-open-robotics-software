package us.ihmc.avatar.pushRecovery;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.function.Consumer;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.AvatarFlatGroundFastWalkingTest;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class HumanoidFootFallDisturbanceRecoveryTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private boolean useExperimentalPhysicsEngine = false;
   private boolean enableToeOffInSingleSupport = false;
   private boolean enableStepAdjustment = false;
   private boolean offsetFootstepsHeightWithExecutionError = false;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      useExperimentalPhysicsEngine = false;
      enableToeOffInSingleSupport = false;
      enableStepAdjustment = false;
      offsetFootstepsHeightWithExecutionError = false;
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void useExperimentalPhysicsEngine()
   {
      useExperimentalPhysicsEngine = true;
   }

   public void enableToeOffInSingleSupport()
   {
      enableToeOffInSingleSupport = true;
   }

   public void enableStepAdjustment()
   {
      enableStepAdjustment = true;
   }

   public void enableOffsetFootstepsHeightWithExecutionError()
   {
      offsetFootstepsHeightWithExecutionError = true;
   }

   public void testBlindWalkOverHole(TestInfo testInfo, double swingDuration, double transferDuration, double holeDepth) throws Exception
   {
      testBlindWalkOverHole(testInfo, swingDuration, transferDuration, holeDepth, null);
   }

   public void testBlindWalkOverHole(TestInfo testInfo, double swingDuration, double transferDuration, double holeDepth, Consumer<YoRegistry> yoVariableMutator)
         throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();

      Point2D holeCenter = new Point2D(1.40, 0.25);
      Vector3D holeSize = new Vector3D(1.10, 0.5, holeDepth);
      FlatGroundWithHoleEnvironment environment = new FlatGroundWithHoleEnvironment(holeCenter, holeSize);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(useExperimentalPhysicsEngine);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + " " + testInfo.getTestMethod().get().getName());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      // Increase area as this test can trigger false positives from the controller failure detection.
      scs.findVariable(WalkingFailureDetectionControlModule.class.getSimpleName(), "icpDistanceFromFootPolygonThreshold").setValueFromDouble(0.10);

      if (enableToeOffInSingleSupport)
      {
         scs.findVariable("doToeOffIfPossibleInSingleSupport").setValueFromDouble(1.0);
         scs.findVariable("forceToeOffAtJointLimit").setValueFromDouble(1.0);
      }

      if (enableStepAdjustment)
      {
         scs.findVariable("controllerMaxReachabilityLength").setValueFromDouble(10.0);
         scs.findVariable("controllerMaxReachabilityWidth").setValueFromDouble(10.0);
      }

      if (yoVariableMutator != null)
         yoVariableMutator.accept(scs.getRootRegistry());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      scs.setInPoint();

      CommonHumanoidReferenceFrames referenceFrames = drcSimulationTestHelper.getReferenceFrames();
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      FramePose3D startPose = new FramePose3D(midFootZUpGroundFrame);
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      FootstepDataListMessage footsteps = forwardSteps(RobotSide.LEFT, 6, 0.5, 0.25, swingDuration, transferDuration, startPose, true);
      footsteps.setOffsetFootstepsHeightWithExecutionError(offsetFootstepsHeightWithExecutionError);
      footsteps.setAreFootstepsAdjustable(enableStepAdjustment);
      drcSimulationTestHelper.publishToController(footsteps);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.3
            * EndToEndTestTools.computeWalkingDuration(footsteps, robotModel.getWalkingControllerParameters())));
   }

   public void testBlindWalkOverStepDown(TestInfo testInfo, double swingDuration, double transferDuration, double stepHeight) throws Exception
   {
      testBlindWalkOverStepDown(testInfo, swingDuration, transferDuration, stepHeight, null);
   }

   public void testBlindWalkOverStepDown(TestInfo testInfo,
                                         double swingDuration,
                                         double transferDuration,
                                         double stepHeight,
                                         Consumer<YoRegistry> yoVariableMutator)
         throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();

      FlatGroundSingleStepEnvironment environment = new FlatGroundSingleStepEnvironment(0.95, -Math.abs(stepHeight));
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(useExperimentalPhysicsEngine);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + " " + testInfo.getTestMethod().get().getName());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      // Increase area as this test can trigger false positives from the controller failure detection.
      scs.findVariable(WalkingFailureDetectionControlModule.class.getSimpleName(), "icpDistanceFromFootPolygonThreshold").setValueFromDouble(0.10);

      if (enableToeOffInSingleSupport)
      {
         scs.findVariable("doToeOffIfPossibleInSingleSupport").setValueFromDouble(1.0);
         scs.findVariable("forceToeOffAtJointLimit").setValueFromDouble(1.0);
      }

      if (enableStepAdjustment)
      {
         scs.findVariable("controllerMaxReachabilityLength").setValueFromDouble(10.0);
         scs.findVariable("controllerMaxReachabilityWidth").setValueFromDouble(10.0);
      }

      if (yoVariableMutator != null)
         yoVariableMutator.accept(scs.getRootRegistry());

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      scs.setInPoint();

      CommonHumanoidReferenceFrames referenceFrames = drcSimulationTestHelper.getReferenceFrames();
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      FramePose3D startPose = new FramePose3D(midFootZUpGroundFrame);
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      FootstepDataListMessage footsteps = forwardSteps(RobotSide.LEFT, 6, 0.5, 0.25, swingDuration, transferDuration, startPose, true);
      footsteps.setOffsetFootstepsHeightWithExecutionError(offsetFootstepsHeightWithExecutionError);
      footsteps.setAreFootstepsAdjustable(enableStepAdjustment);
      drcSimulationTestHelper.publishToController(footsteps);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1
            * EndToEndTestTools.computeWalkingDuration(footsteps, robotModel.getWalkingControllerParameters())));
   }

   private static FootstepDataListMessage forwardSteps(RobotSide initialStepSide,
                                                       int numberOfSteps,
                                                       double stepLength,
                                                       double stepWidth,
                                                       double swingTime,
                                                       double transferTime,
                                                       Pose3DReadOnly startPose,
                                                       boolean squareUp)
   {
      return AvatarFlatGroundFastWalkingTest.forwardSteps(initialStepSide,
                                                          numberOfSteps,
                                                          i -> stepLength,
                                                          stepWidth,
                                                          swingTime,
                                                          transferTime,
                                                          startPose,
                                                          squareUp);
   }

   private static class FlatGroundWithHoleEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D("flatGroundWithHole");

      public FlatGroundWithHoleEnvironment(Point2D holeCenter, Vector3D holeSize)
      {
         double thickness = 0.01;
         Vector2D groundSize = new Vector2D(100.0, 100.0);

         // Fill bottom of the hole
         double x0 = holeCenter.getX() - 0.5 * groundSize.getX();
         double y0 = holeCenter.getY() + 0.5 * groundSize.getY();
         double xf = holeCenter.getX() + 0.5 * groundSize.getX();
         double yf = holeCenter.getY() - 0.5 * groundSize.getY();
         double z0 = -thickness - holeSize.getZ();
         double zf = -holeSize.getZ();
         combinedTerrainObject3D.addBox(x0, y0, xf, yf, z0, zf, YoAppearance.Orange());

         z0 = -holeSize.getZ();
         zf = 0.0;

         // Block left to the hole
         x0 = holeCenter.getX() - 0.5 * groundSize.getX();
         xf = holeCenter.getX() + 0.5 * groundSize.getX();
         y0 = holeCenter.getY() + 0.5 * groundSize.getY();
         yf = holeCenter.getY() + 0.5 * holeSize.getY();
         combinedTerrainObject3D.addBox(x0, y0, xf, yf, z0, zf);

         // Block right to the hole
         x0 = holeCenter.getX() - 0.5 * groundSize.getX();
         xf = holeCenter.getX() + 0.5 * groundSize.getX();
         y0 = holeCenter.getY() - 0.5 * holeSize.getY();
         yf = holeCenter.getY() - 0.5 * groundSize.getY();
         combinedTerrainObject3D.addBox(x0, y0, xf, yf, z0, zf);

         // Block front of the hole
         x0 = holeCenter.getX() - 0.5 * groundSize.getX();
         xf = holeCenter.getX() - 0.5 * holeSize.getX();
         y0 = holeCenter.getY() + 0.5 * holeSize.getY();
         yf = holeCenter.getY() - 0.5 * holeSize.getY();
         combinedTerrainObject3D.addBox(x0, y0, xf, yf, z0, zf);

         // Block back of the hole
         x0 = holeCenter.getX() + 0.5 * holeSize.getX();
         xf = holeCenter.getX() + 0.5 * groundSize.getX();
         y0 = holeCenter.getY() + 0.5 * holeSize.getY();
         yf = holeCenter.getY() - 0.5 * holeSize.getY();
         combinedTerrainObject3D.addBox(x0, y0, xf, yf, z0, zf);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return combinedTerrainObject3D;
      }
   }

   private static class FlatGroundSingleStepEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D combinedTerrainObject3D = new CombinedTerrainObject3D("flatGroundWithHole");

      public FlatGroundSingleStepEnvironment(double stepX, double stepHeight)
      {
         double thickness = 0.01;
         Vector2D groundSize = new Vector2D(100.0, 100.0);
         double xStart = stepX - 0.5 * groundSize.getX();
         double yStart = +0.5 * groundSize.getY();
         double xEnd = stepX + 0.5 * groundSize.getX();
         double yEnd = -0.5 * groundSize.getY();
         double zStart = -thickness + (stepHeight > 0.0 ? 0.0 : stepHeight);
         double zEnd = +(stepHeight > 0.0 ? 0.0 : stepHeight);
         combinedTerrainObject3D.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd, YoAppearance.Orange());

         xStart = stepX - (stepHeight > 0.0 ? 0.0 : 0.5 * groundSize.getX());
         xEnd = stepX + (stepHeight > 0.0 ? 0.5 * groundSize.getX() : 0.0);
         zStart = (stepHeight > 0.0 ? 0.0 : stepHeight);
         zEnd = (stepHeight > 0.0 ? stepHeight : 0.0);
         combinedTerrainObject3D.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return combinedTerrainObject3D;
      }
   }
}
