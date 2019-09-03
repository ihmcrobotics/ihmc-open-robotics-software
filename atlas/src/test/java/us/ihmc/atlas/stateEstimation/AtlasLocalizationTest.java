package us.ihmc.atlas.stateEstimation;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import controller_msgs.msg.dds.StampedPosePacket;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public class AtlasLocalizationTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private static final double EPSILON = 1.0e-3;

   private DRCSimulationTestHelper testHelper;

   @Test
   public void testPoseAdjustment() throws SimulationExceededMaximumTimeException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_FOREARMS);

      PelvisPoseCorrectionCommunicator pelvisPoseCorrectionCommunicator = new PelvisPoseCorrectionCommunicator();

      testHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      testHelper.setExternalPelvisCorrectorSubscriber(pelvisPoseCorrectionCommunicator);
      testHelper.createSimulation("LocalizationTest");

      testHelper.simulateAndBlockAndCatchExceptions(1.0);

      FramePose3D pelvisPose = new FramePose3D();
      ReferenceFrame pelvisFrame = testHelper.getReferenceFrames().getPelvisFrame();

      pelvisPose.setToZero(pelvisFrame);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      System.out.println("Started at " + pelvisPose.getX());

      pelvisPose.setToZero(pelvisFrame);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      pelvisPose.setX(0.1);

      StampedPosePacket posePacket = new StampedPosePacket();
      posePacket.setConfidenceFactor(1.0);
      posePacket.getPose().set(pelvisPose);
      for (int i = 0; i < 5; i++)
      {
         long timestamp = Conversions.secondsToNanoseconds(testHelper.getSimulationConstructionSet().getTime() - robotModel.getEstimatorDT());
         posePacket.setTimestamp(timestamp);
         pelvisPoseCorrectionCommunicator.set(posePacket);
         testHelper.simulateAndBlockAndCatchExceptions(1.0);
      }

      pelvisPose.setToZero(pelvisFrame);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      double deadzone = testHelper.getYoVariable("xDeadzoneSize").getValueAsDouble();
      System.out.println("Ended at " + pelvisPose.getX());
      System.out.println("Should be at " + posePacket.getPose().getX() + " with deadzone of " + deadzone);
      EuclidGeometryTestTools.assertPose3DEquals(posePacket.getPose(), pelvisPose, deadzone + EPSILON);
   }

   private class PelvisPoseCorrectionCommunicator implements PelvisPoseCorrectionCommunicatorInterface
   {
      private StampedPosePacket packet;

      public void set(StampedPosePacket packet)
      {
         this.packet = packet;
      }

      @Override
      public boolean hasNewPose()
      {
         return packet != null;
      }

      @Override
      public StampedPosePacket getNewExternalPose()
      {
         StampedPosePacket ret = packet;
         packet = null;
         return ret;
      }

      @Override
      public void receivedPacket(StampedPosePacket object)
      {
      }

      @Override
      public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket)
      {
      }

      @Override
      public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
      {
      }
   }

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

      if (testHelper != null)
      {
         testHelper.destroySimulation();
         testHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
