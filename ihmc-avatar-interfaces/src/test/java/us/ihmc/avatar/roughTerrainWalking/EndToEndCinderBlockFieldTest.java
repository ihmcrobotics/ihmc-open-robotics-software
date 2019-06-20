package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CinderBlockFieldEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Tag("video")
public abstract class EndToEndCinderBlockFieldTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper simulationTestHelper;

   public abstract double getStepHeightOffset();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
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
      if (simulationTestHelper != null)
      {
         simulationTestHelper.destroySimulation();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testWalkingOverCinderBlockField() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      FootstepDataListMessage footsteps = generateFootstepsForCinderBlockField(cinderBlockFieldEnvironment.getCinderBlockPoses(), getStepHeightOffset());

      simulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      simulationTestHelper.setTestEnvironment(cinderBlockFieldEnvironment);
      simulationTestHelper.createSimulation("EndToEndCinderBlockFieldTest");

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      FramePoint3D pelvisPosition = new FramePoint3D(fullRobotModel.getPelvis().getBodyFixedFrame());
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPosition.add(0.0, 0.0, getPelvisOffsetHeight());
      double desiredHeight = pelvisPosition.getZ();
      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, desiredHeight));

      simulationTestHelper.publishToController(footsteps);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      success = simulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.getFootstepDataList().size() * stepTime + 2.0 * initialFinalTransfer + 1.0);
      assertTrue(success);

      Point3D step1 = footsteps.getFootstepDataList().get(footsteps.getFootstepDataList().size() - 1).getLocation();
      Point3D step2 = footsteps.getFootstepDataList().get(footsteps.getFootstepDataList().size() - 2).getLocation();
      Point3D expectedPelvis = new Point3D();
      expectedPelvis.interpolate(step1, step2, 0.5);
      expectedPelvis.setZ(desiredHeight);
      Vector3D margin = new Vector3D(0.25, 0.25, 0.25);
      Point3D min = new Point3D(expectedPelvis);
      Point3D max = new Point3D(expectedPelvis);
      min.sub(margin);
      max.add(margin);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(new BoundingBox3D(min, max));

      simulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   public abstract double getPelvisOffsetHeight();

   private static FootstepDataListMessage generateFootstepsForCinderBlockField(List<List<FramePose3D>> cinderBlockPoses, double zOffset)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      int numberOfColumns = cinderBlockPoses.get(0).size();

      int indexForLeftSide = (numberOfColumns - 1) / 2;
      int indexForRightSide = indexForLeftSide + 1;
      SideDependentList<List<FramePose3D>> columns = extractColumns(cinderBlockPoses, indexForLeftSide, indexForRightSide);

      for (int row = 0; row < cinderBlockPoses.size(); row++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePose3D cinderBlockPose = columns.get(robotSide).get(row);
            Point3D location = new Point3D();
            Quaternion orientation = new Quaternion();
            cinderBlockPose.get(location, orientation);
            location.setZ(location.getZ() + zOffset);
            FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
            footsteps.getFootstepDataList().add().set(footstep);
         }
      }

      return footsteps;
   }

   private static SideDependentList<List<FramePose3D>> extractColumns(List<List<FramePose3D>> cinderBlockPoses, int indexForLeftSide, int indexForRightSide)
   {
      SideDependentList<Integer> columnIndices = new SideDependentList<Integer>(indexForLeftSide, indexForRightSide);
      SideDependentList<List<FramePose3D>> sideDependentColumns = new SideDependentList<List<FramePose3D>>(new ArrayList<FramePose3D>(), new ArrayList<FramePose3D>());

      for (RobotSide robotSide : RobotSide.values)
      {
         int column = columnIndices.get(robotSide);

         for (int row = 0; row < cinderBlockPoses.size(); row++)
            sideDependentColumns.get(robotSide).add(cinderBlockPoses.get(row).get(column));
      }

      return sideDependentColumns;
   }
}
