package us.ihmc.avatar.pushRecovery;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.AvatarFlatGroundFastWalkingTest;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
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

public abstract class HumanoidFootFallDisturbanceRecoveryTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private boolean useExperimentalPhysicsEngine = false;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      useExperimentalPhysicsEngine = false;
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      //      if (simulationTestingParameters.getKeepSCSUp())
      ThreadTools.sleepForever();

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void setUseExperimentalPhysicsEngine(boolean useExperimentalPhysicsEngine)
   {
      this.useExperimentalPhysicsEngine = useExperimentalPhysicsEngine;
   }

   public void testBlindWalkOverHole(TestInfo testInfo, double swingDuration, double transferDuration) throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();

      Point2D holeCenter = new Point2D(1.40, 0.25);
      Vector3D holeSize = new Vector3D(0.75, 0.5, 0.10);
      FlatGroundWithHoleEnvironment environment = new FlatGroundWithHoleEnvironment(holeCenter, holeSize);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(useExperimentalPhysicsEngine);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + " " + testInfo.getTestMethod().get().getName());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      scs.setInPoint();

      CommonHumanoidReferenceFrames referenceFrames = drcSimulationTestHelper.getReferenceFrames();
      MovingReferenceFrame midFootZUpGroundFrame = referenceFrames.getMidFootZUpGroundFrame();
      FramePose3D startPose = new FramePose3D(midFootZUpGroundFrame);
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      FootstepDataListMessage footsteps = forwardSteps(RobotSide.LEFT, 6, 0.5, 0.25, swingDuration, transferDuration, startPose, true);
//      footsteps.setAreFootstepsAdjustable(true);
      drcSimulationTestHelper.publishToController(footsteps);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1
            * EndToEndTestTools.computeWalkingDuration(footsteps, robotModel.getWalkingControllerParameters())));
   }

   private static FootstepDataListMessage forwardSteps(RobotSide initialStepSide, int numberOfSteps, double stepLength, double stepWidth, double swingTime,
                                                       double transferTime, Pose3DReadOnly startPose, boolean squareUp)
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
         double xStart = holeCenter.getX() - 0.5 * groundSize.getX();
         double yStart = holeCenter.getY() + 0.5 * groundSize.getY();
         double xEnd = holeCenter.getX() + 0.5 * groundSize.getX();
         double yEnd = holeCenter.getY() - 0.5 * groundSize.getY();
         double zStart = -thickness - holeSize.getZ();
         double zEnd = -holeSize.getZ();
         combinedTerrainObject3D.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd, YoAppearance.Orange());

         xEnd = holeCenter.getX() - 0.5 * holeSize.getX();
         yEnd = holeCenter.getY() - 0.5 * groundSize.getY();
         zStart = -holeSize.getZ();
         zEnd = 0.0;
         combinedTerrainObject3D.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         xStart = holeCenter.getX() + 0.5 * holeSize.getX();
         yStart = holeCenter.getY() + 0.5 * groundSize.getY();
         xEnd = holeCenter.getX() + 0.5 * groundSize.getX();
         yEnd = holeCenter.getY() - 0.5 * groundSize.getY();
         combinedTerrainObject3D.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         xStart = holeCenter.getX() - 0.5 * holeSize.getX();
         yStart = holeCenter.getY() + 0.5 * groundSize.getY();
         xEnd = holeCenter.getX() + 0.5 * holeSize.getX();
         yEnd = holeCenter.getY() + 0.5 * holeSize.getY();
         combinedTerrainObject3D.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         xStart = holeCenter.getX() - 0.5 * groundSize.getX();
         yStart = holeCenter.getY() - 0.5 * holeSize.getY();
         xEnd = holeCenter.getX() + 0.5 * groundSize.getX();
         yEnd = holeCenter.getY() - 0.5 * groundSize.getY();
         combinedTerrainObject3D.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return combinedTerrainObject3D;
      }
   }
}
