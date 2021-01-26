package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.physics.ContactParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class HumanoidEndToEndSlopeTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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

   public void testSlope(TestInfo testInfo, boolean up, double swingDuration, double transferDuration, double maxStepLength, double heightOffset,
                         double torsoPitch, boolean useExperimentalPhysicsEngine)
         throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      double footLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getActualFootLength();
      double slopeAngle = Math.toRadians(30.0);
      double slopeLength = 3.0;
      double topZ = Math.tan(slopeAngle) * slopeLength;
      double startX = (up ? 0.0 : 1.2 + slopeLength) + 0.3;
      double startZ = up ? 0.0 : topZ;

      SlopeEnvironment environment = new SlopeEnvironment(slopeAngle, 1.5, slopeLength);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setStartingLocation(new OffsetAndYawRobotInitialSetup(startX, 0, startZ));
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(useExperimentalPhysicsEngine);
      ContactParameters contactParameters = ContactParameters.defaultIneslasticContactParameters(true);
      contactParameters.setCoulombMomentFrictionRatio(0.6);
      drcSimulationTestHelper.getSCSInitialSetup().setExperimentalPhysicsEngineContactParameters(contactParameters);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + " " + testInfo.getTestMethod().get().getName());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setCameraFix(startX, 0.0, 0.8 + startZ);
      scs.setCameraPosition(startX, -slopeLength, 0.8 + startZ);
      scs.setCameraTrackingOffsets(0.0, 0.0, 0.0);
      scs.setCameraDollyOffsets(up ? -2.4 : 2.4, -6.0, 0.0);
      scs.setCameraTracking(true, true, true, true);
      scs.setCameraDolly(true, true, true, true);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      publishHeightOffset(heightOffset);
      pitchTorsoForward(torsoPitch);

      double slopeStartX = up ? 0.6 : 1.8 + slopeLength;
      double slopeEndX = slopeStartX + slopeLength;
      double slopeStartZ = up ? 0.0 : topZ;
      double stanceWidth = 0.30;
      FootstepDataListMessage footsteps = createSlopeFootsteps(slopeStartX,
                                                               slopeEndX,
                                                               slopeStartZ,
                                                               up ? -slopeAngle : slopeAngle,
                                                               footLength,
                                                               stanceWidth,
                                                               maxStepLength);
      computeSwingWaypoints(robotModel, footsteps);
      setStepDurations(footsteps, swingDuration, transferDuration);

      scs.setInPoint();

      publishFootstepsAndSimulate(robotModel, footsteps);
   }

   private void publishHeightOffset(double heightOffset) throws Exception
   {
      if (!Double.isFinite(heightOffset) || EuclidCoreTools.epsilonEquals(0.0, heightOffset, 1.0e-3))
         return;

      MovingReferenceFrame rootJointFrame = drcSimulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      double z = rootJointFrame.getTransformToRoot().getTranslationZ();
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, z + heightOffset));
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
   }

   private void pitchTorsoForward(double angle) throws Exception
   {
      if (!Double.isFinite(angle) || EuclidCoreTools.epsilonEquals(0.0, angle, 1.0e-3))
         return;

      ChestTrajectoryMessage message = HumanoidMessageTools.createChestTrajectoryMessage(0.5,
                                                                                         new YawPitchRoll(0.0, angle, 0.0),
                                                                                         ReferenceFrame.getWorldFrame());
      drcSimulationTestHelper.publishToController(message);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
   }

   private void computeSwingWaypoints(DRCRobotModel robotModel, FootstepDataListMessage footsteps)
   {
      double swingHeight = 1.5 * robotModel.getWalkingControllerParameters().getSteppingParameters().getDefaultSwingHeightFromStanceFoot();

      for (int i = 2; i < footsteps.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage previousFootstep = footsteps.getFootstepDataList().get(i - 2);
         FootstepDataMessage footstep = footsteps.getFootstepDataList().get(i);
         Point3D start = new Point3D(previousFootstep.getLocation());
         Point3D end = new Point3D(footstep.getLocation());
         Point3D firstWaypoint = new Point3D();
         Point3D secondWaypoint = new Point3D();
         firstWaypoint.interpolate(start, end, 0.25);
         secondWaypoint.interpolate(start, end, 0.90);
         Vector3D startNormal = new Vector3D(Axis3D.Z);
         previousFootstep.getOrientation().transform(startNormal);
         Vector3D endNormal = new Vector3D(Axis3D.Z);
         footstep.getOrientation().transform(endNormal);

         firstWaypoint.scaleAdd(swingHeight, startNormal, firstWaypoint);
         secondWaypoint.scaleAdd(swingHeight, endNormal, secondWaypoint);

         footstep.getCustomPositionWaypoints().add().set(firstWaypoint);
         footstep.getCustomPositionWaypoints().add().set(secondWaypoint);
         footstep.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
      }
   }

   private static FootstepDataListMessage setStepDurations(FootstepDataListMessage message, double swingDuration, double transferDuration)
   {
      if (Double.isFinite(swingDuration) && swingDuration > 0.0)
      {
         for (int i = 0; i < message.getFootstepDataList().size(); i++)
         {
            message.getFootstepDataList().get(i).setSwingDuration(swingDuration);
         }
      }

      if (Double.isFinite(transferDuration) && transferDuration > 0.0)
      {
         for (int i = 0; i < message.getFootstepDataList().size(); i++)
         {
            message.getFootstepDataList().get(i).setTransferDuration(transferDuration);
         }
      }

      return message;
   }

   private void publishFootstepsAndSimulate(DRCRobotModel robotModel, FootstepDataListMessage footsteps) throws Exception
   {
      double walkingDuration = EndToEndTestTools.computeWalkingDuration(footsteps, robotModel.getWalkingControllerParameters());
      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.1 * walkingDuration));
   }

   private static FootstepDataListMessage createSlopeFootsteps(double xSlopeStart, double xSlopeEnd, double zSlopeStart, double slopeAngle, double footLength,
                                                               double stanceWidth, double maxStepLength)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      double margin = 0.035;

      for (RobotSide stepSide : RobotSide.values)
      {
         double x = xSlopeStart - 0.5 * footLength - margin;
         double y = 0.5 * stepSide.negateIfRightSide(stanceWidth);
         double z = zSlopeStart;

         FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(x, y, z);
      }

      double slopeLength = (xSlopeEnd - xSlopeStart) / Math.cos(slopeAngle);
      int numberOfSteps = (int) Math.ceil((slopeLength - footLength - 2.0 * margin) / maxStepLength) + 1;
      double stepLength = (slopeLength - footLength - 2.0 * margin) / (numberOfSteps - 1.0);
      assertTrue(stepLength <= maxStepLength);

      Pose3D footstepPose = new Pose3D();
      footstepPose.appendTranslation(xSlopeStart, 0.0, zSlopeStart);
      footstepPose.appendPitchRotation(slopeAngle);
      footstepPose.appendTranslation(0.5 * footLength + margin, 0.0, 0.0);

      RobotSide stepSide = RobotSide.LEFT;

      for (int i = 0; i < numberOfSteps; i++)
      {
         footstepPose.getPosition().setY(stepSide.negateIfRightSide(0.5 * stanceWidth));

         FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(footstepPose.getPosition());
         footstep.getOrientation().set(footstepPose.getOrientation());

         footstepPose.appendTranslation(stepLength, 0.0, 0.0);

         stepSide = stepSide.getOppositeSide();
      }

      double zSlopeEnd = zSlopeStart - slopeLength * Math.sin(slopeAngle);

      for (int i = 0; i < 2; i++)
      {
         double x = xSlopeEnd + 0.5 * footLength + margin;
         double y = 0.5 * stepSide.negateIfRightSide(stanceWidth);
         double z = zSlopeEnd;

         FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
         footstep.setRobotSide(stepSide.toByte());
         footstep.getLocation().set(x, y, z);

         stepSide = stepSide.getOppositeSide();
      }

      return footsteps;
   }

   private static class SlopeEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Slope");

      public SlopeEnvironment(double slopeAngle, double slopeWidth, double slopeLength)
      {
         double startingBlockLength = 1.2;
         double xStart = -0.5 * startingBlockLength;
         double xEnd = 0.5 * startingBlockLength + 0 * slopeLength;
         double yStart = -0.5 * slopeWidth;
         double yEnd = 0.5 * slopeWidth;
         double zStart = -0.10;
         double zEnd = 0.0;
         terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         xStart = 0.5 * startingBlockLength;
         xEnd = 0.5 * startingBlockLength + slopeLength;
         double height = slopeLength * Math.tan(slopeAngle);
         terrainObject.addRamp(xStart, yStart, xEnd, yEnd, height, YoAppearance.Orange());

         xStart = xEnd;
         xEnd += startingBlockLength;
         zEnd = height;
         terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);

         xStart = xEnd + slopeLength;
         terrainObject.addRamp(xStart, yStart, xEnd, yEnd, height, YoAppearance.AliceBlue());

         xEnd = xStart + startingBlockLength;
         xStart -= slopeLength;
         zEnd = 0.0;
         terrainObject.addBox(xStart, yStart, xEnd, yEnd, zStart, zEnd);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrainObject;
      }
   }
}
