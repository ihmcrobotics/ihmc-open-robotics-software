package us.ihmc.gr00t;

import org.junit.jupiter.api.Test;

import java.nio.DoubleBuffer;
import java.util.List;
import java.util.function.Consumer;

import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertThrows;

class Gr00tHumanoidTest
{
   private static final int HAND_TARGET_COUNT = 6;
   private static final Gr00tBimanualActionDecoder DECODER = new Gr00tBimanualActionDecoder(HAND_TARGET_COUNT);
   private static final int ACTION_SIZE = DECODER.getActionSize();
   private static final Gr00tHumanoidConfiguration CONFIGURATION = new Gr00tHumanoidConfiguration(RobotSide.RIGHT,
                                                                                                  false,
                                                                                                  true,
                                                                                                  NeckJointName.DISTAL_NECK_YAW,
                                                                                                  NeckJointName.DISTAL_NECK_PITCH,
                                                                                                  0.1,
                                                                                                  6.0,
                                                                                                  3.0,
                                                                                                  0.2,
                                                                                                  1.0,
                                                                                                  0.5,
                                                                                                  2.0,
                                                                                                  0.025,
                                                                                                  0.15);

   @Test
   void disconnectedBridgeRetriesOnlyAfterTheBackoffDeadline()
   {
      long deadline = 5_000_000_000L;
      assertFalse(Gr00tInferenceSession.isRetryDue(deadline - 1L, deadline));
      assertTrue(Gr00tInferenceSession.isRetryDue(deadline, deadline));
      assertTrue(Gr00tInferenceSession.isRetryDue(deadline + 1L, deadline));
   }

   @Test
   void extractsEveryValidPolicyRowInOrder()
   {
      DoubleBuffer chunk = DoubleBuffer.allocate(6 * ACTION_SIZE);
      for (int dimension = 0; dimension < ACTION_SIZE; dimension++)
         chunk.put(dimension, Double.NaN); // Invalid leading row must be skipped.

      double[] rightX = {0.30, 0.10, 0.20, 0.40, 0.00};
      double[] neckY = {0.25, 0.05, 0.15, 0.35, -0.05};
      for (int sample = 0; sample < rightX.length; sample++)
      {
         int row = sample + 1;
         int offset = row * ACTION_SIZE;
         for (int dimension = 0; dimension < ACTION_SIZE; dimension++)
            chunk.put(offset + dimension, 0.0);

         chunk.put(offset, -0.2);
         chunk.put(offset + 1, 0.3);
         chunk.put(offset + 2, 1.0);
         chunk.put(offset + 6, sample % 2 == 0 ? 1.0 : -1.0);
         chunk.put(offset + 7, rightX[sample]);
         chunk.put(offset + 8, -0.35);
         chunk.put(offset + 9, 0.95);
         chunk.put(offset + 13, sample % 2 == 0 ? -1.0 : 1.0);
         chunk.put(offset + 14, neckY[sample]);
         chunk.put(offset + 15, 0.4);
      }

      List<Gr00tHumanoidAction> candidates = DECODER.decode(chunk, 6);

      assertEquals(5, candidates.size());
      assertEquals(0.30, candidates.get(0).getWristPose(RobotSide.RIGHT).getPosition().getX(), 1.0e-12);
      assertEquals(0.25, candidates.get(0).getNeckPitch(), 1.0e-12);
      assertEquals(0.00, candidates.get(4).getWristPose(RobotSide.RIGHT).getPosition().getX(), 1.0e-12);
      assertEquals(-0.05, candidates.get(4).getNeckPitch(), 1.0e-12);
      assertEquals(1.0, Math.abs(candidates.get(0).getWristPose(RobotSide.RIGHT).getOrientation().getS()), 1.0e-12);
   }

   @Test
   void preservesAllSixteenActionsAcrossMotionChanges()
   {
      DoubleBuffer chunk = DoubleBuffer.allocate(16 * ACTION_SIZE);
      for (int row = 0; row < 16; row++)
      {
         int offset = row * ACTION_SIZE;
         for (int dimension = 0; dimension < ACTION_SIZE; dimension++)
            chunk.put(offset + dimension, 0.0);
         chunk.put(offset + 6, 1.0);
         chunk.put(offset + 7, 0.46);
         chunk.put(offset + 8, -0.54);
         chunk.put(offset + 9, row < 11 ? 1.31 : 1.16);
         chunk.put(offset + 13, 1.0);
      }

      List<Gr00tHumanoidAction> candidates = DECODER.decode(chunk, 16);

      assertEquals(16, candidates.size());
      assertEquals(1.31, candidates.get(0).getWristPose(RobotSide.RIGHT).getPosition().getZ(), 1.0e-12);
      assertEquals(1.31, candidates.get(10).getWristPose(RobotSide.RIGHT).getPosition().getZ(), 1.0e-12);
      assertEquals(1.16, candidates.get(11).getWristPose(RobotSide.RIGHT).getPosition().getZ(), 1.0e-12);
      assertEquals(1.16, candidates.get(15).getWristPose(RobotSide.RIGHT).getPosition().getZ(), 1.0e-12);
   }

   @Test
   void rejectsChunksWithoutAnyFiniteNormalizedWristPose()
   {
      DoubleBuffer chunk = DoubleBuffer.allocate(ACTION_SIZE);
      for (int dimension = 0; dimension < ACTION_SIZE; dimension++)
         chunk.put(dimension, 0.0);

      assertEquals(0, DECODER.decode(chunk, 1).size());
   }

   @Test
   void humanoidActionsDefensivelyCopyPosesAndHands()
   {
      var pose = new us.ihmc.euclid.geometry.Pose3D();
      double[] hand = {1.0};
      Gr00tHumanoidAction action = new Gr00tHumanoidAction(pose, pose, 0.15, 0.4, hand, hand);
      pose.setX(2.0);
      hand[0] = 2.0;
      assertEquals(0.0, action.getWristPose(RobotSide.RIGHT).getX(), 1.0e-12);
      assertEquals(1.0, action.getHandTargets(RobotSide.RIGHT)[0], 1.0e-12);
      double[] returnedHand = action.getHandTargets(RobotSide.RIGHT);
      returnedHand[0] = 3.0;
      assertEquals(1.0, action.getHandTargets(RobotSide.RIGHT)[0], 1.0e-12);
   }

   @Test
   void preservesEveryLearnedNeckTargetOnArmTimestamps()
   {
      double[] times = {0.0, 1.0, 2.0, 3.0};
      double[][] neckZY = {{-0.1, 0.0}, {-0.2, 0.1}, {-0.3, 0.2}, {-0.4, 0.3}};

      var message = Gr00tHumanoidTrajectoryTools.createNeckTrajectoryMessage(times, neckZY);

      assertEquals(2, message.getJointspaceTrajectory().getJointTrajectoryMessages().size());
      for (int joint = 0; joint < 2; joint++)
      {
         var points = message.getJointspaceTrajectory().getJointTrajectoryMessages().get(joint).getTrajectoryPoints();
         assertEquals(3, points.size());
         for (int sample = 1; sample < times.length; sample++)
         {
            assertEquals(times[sample], points.get(sample - 1).getTime(), 1.0e-12);
            assertEquals(neckZY[sample][joint], points.get(sample - 1).getPosition(), 1.0e-12);
         }
         assertEquals(0.0, points.get(2).getVelocity(), 1.0e-12);
      }
   }

   @Test
   void trajectoryDurationHonorsStartupMinimumsAndVelocityLimits()
   {
      assertEquals(6.0, Gr00tHumanoidTrajectoryTools.computeTrajectoryDuration(CONFIGURATION, true, 0.4, 0.5), 1.0e-12);
      assertEquals(7.0, Gr00tHumanoidTrajectoryTools.computeTrajectoryDuration(CONFIGURATION, true, 1.4, 0.5), 1.0e-12);
      assertEquals(3.0, Gr00tHumanoidTrajectoryTools.computeTrajectoryDuration(CONFIGURATION, false, 0.2, 0.5), 1.0e-12);
      assertEquals(4.0, Gr00tHumanoidTrajectoryTools.computeTrajectoryDuration(CONFIGURATION, false, 0.2, 4.0), 1.0e-12);
   }

   @Test
   void replanningWaitsForTrajectoryAndSettleTime()
   {
      assertEquals(6.5, Gr00tHumanoidTrajectoryTools.stageHoldRemainingSeconds(CONFIGURATION, 0.0, 6.0), 1.0e-12);
      assertEquals(0.5, Gr00tHumanoidTrajectoryTools.stageHoldRemainingSeconds(CONFIGURATION, 6.0, 6.0), 1.0e-12);
      assertEquals(0.0, Gr00tHumanoidTrajectoryTools.stageHoldRemainingSeconds(CONFIGURATION, 6.5, 6.0), 1.0e-12);
      assertEquals(0.0, Gr00tHumanoidTrajectoryTools.stageHoldRemainingSeconds(CONFIGURATION, 7.0, 6.0), 1.0e-12);
   }

   @Test
   void actionChunkCannotRemainLatchedForeverOnCartesianTrackingError()
   {
      assertFalse(Gr00tHumanoidTrajectoryTools.isActionChunkComplete(CONFIGURATION, 6.49, 6.0, true));
      assertTrue(Gr00tHumanoidTrajectoryTools.isActionChunkComplete(CONFIGURATION, 6.5, 6.0, true));

      assertFalse(Gr00tHumanoidTrajectoryTools.isActionChunkComplete(CONFIGURATION, 6.5, 6.0, false));
      assertFalse(Gr00tHumanoidTrajectoryTools.isActionChunkComplete(CONFIGURATION, 8.49, 6.0, false));
      assertTrue(Gr00tHumanoidTrajectoryTools.isActionChunkComplete(CONFIGURATION, 8.5, 6.0, false));
      assertEquals(2.0, Gr00tHumanoidTrajectoryTools.endpointConfirmationRemainingSeconds(CONFIGURATION, 6.5, 6.0), 1.0e-12);
      assertEquals(0.0, Gr00tHumanoidTrajectoryTools.endpointConfirmationRemainingSeconds(CONFIGURATION, 8.5, 6.0), 1.0e-12);
   }

   @Test
   void learnedHandTargetsFollowTheirMatchingArmWaypoints()
   {
      double[] trajectoryTimes = {0.0, 1.0, 2.0, 3.0};
      assertEquals(0, Gr00tHumanoidTrajectoryTools.latestDueAction(trajectoryTimes, 0.99));
      assertEquals(1, Gr00tHumanoidTrajectoryTools.latestDueAction(trajectoryTimes, 1.0));
      assertEquals(2, Gr00tHumanoidTrajectoryTools.latestDueAction(trajectoryTimes, 2.5));
      assertEquals(3, Gr00tHumanoidTrajectoryTools.latestDueAction(trajectoryTimes, 4.0));
   }

   @Test
   void endpointConvergenceRequiresBothPositionAndOrientation()
   {
      us.ihmc.euclid.geometry.Pose3D current = new us.ihmc.euclid.geometry.Pose3D();
      us.ihmc.euclid.geometry.Pose3D target = new us.ihmc.euclid.geometry.Pose3D();
      target.getPosition().set(0.02, 0.0, 0.0);
      target.getOrientation().setYawPitchRoll(0.10, 0.0, 0.0);
      assertTrue(Gr00tHumanoidTrajectoryTools.isEndpointConverged(CONFIGURATION, current, target));

      target.getPosition().setX(0.03);
      assertFalse(Gr00tHumanoidTrajectoryTools.isEndpointConverged(CONFIGURATION, current, target));
      target.getPosition().setX(0.0);
      target.getOrientation().setYawPitchRoll(0.20, 0.0, 0.0);
      assertFalse(Gr00tHumanoidTrajectoryTools.isEndpointConverged(CONFIGURATION, current, target));
   }

   @Test
   void validatesPublicConfigurationAtConstruction()
   {
      assertThrows(IllegalArgumentException.class,
                   () -> new Gr00tModelConfiguration("layout", "id", 0, 28, 16, 10.0, 640, 480, 50, "left", "right"));
      assertThrows(IllegalArgumentException.class,
                   () -> new Gr00tModelConfiguration("layout", "id", 28, 28, 51, 10.0, 640, 480, 50, "left", "right"));
      assertThrows(IllegalArgumentException.class,
                   () -> new Gr00tHumanoidConfiguration(RobotSide.RIGHT,
                                                        false,
                                                        true,
                                                        null,
                                                        null,
                                                        0.0,
                                                        6.0,
                                                        3.0,
                                                        0.2,
                                                        1.0,
                                                        0.5,
                                                        2.0,
                                                        0.025,
                                                        0.15));
   }

   @Test
   void semanticNeckMappingAndJointLimitClampingAreRobotConfigured()
   {
      NeckJointName[] neckJoints = {NeckJointName.DISTAL_NECK_PITCH, NeckJointName.DISTAL_NECK_YAW};
      assertEquals(1, Gr00tHumanoidArmPlanner.findNeckJointIndex(neckJoints, NeckJointName.DISTAL_NECK_YAW));
      assertEquals(0, Gr00tHumanoidArmPlanner.findNeckJointIndex(neckJoints, NeckJointName.DISTAL_NECK_PITCH));
      assertEquals(-1, Gr00tHumanoidArmPlanner.findNeckJointIndex(neckJoints, null));
      assertEquals(0.5, Gr00tHumanoidArmPlanner.clampToLimits(0.75, -0.5, 0.5), 1.0e-12);
      assertEquals(-0.5, Gr00tHumanoidArmPlanner.clampToLimits(-0.75, -0.5, 0.5), 1.0e-12);
   }

   @Test
   void noOpHandsSupportEmbodimentsWithoutHandHardware()
   {
      Gr00tHandController hands = Gr00tHandController.noOp();
      hands.setEnabled(true);
      assertTrue(hands.isEnabled());
      assertFalse(hands.hasState(RobotSide.RIGHT));
      assertFalse(hands.publishPolicyTargets(RobotSide.RIGHT, new double[0]));
      assertEquals(0, hands.getJointPositions(RobotSide.RIGHT).length);
   }

   @Test
   void staleInferenceResponsesAreRejectedByTheRuntime()
   {
      FakeTask task = new FakeTask();
      assertTrue(Gr00tInferenceSession.isResponseAccepted(task, new Gr00tTask.Request("command", 2L)));
      assertFalse(Gr00tInferenceSession.isResponseAccepted(task, new Gr00tTask.Request("command", 1L)));
      assertFalse(Gr00tInferenceSession.isResponseAccepted(task, null));
   }

   private static final class FakeTask implements Gr00tTask
   {
      @Override public Gr00tModelConfiguration getModelConfiguration() { return null; }
      @Override public Gr00tObservationSource createObservationSource(Gr00tClient client) { return null; }
      @Override public void setStatusConsumer(Consumer<String> statusConsumer) { }
      @Override public void updateBeforeInference() { }
      @Override public void updateAfterInference() { }
      @Override public boolean isRunning() { return true; }
      @Override public boolean shouldRequestInference() { return true; }
      @Override public Request getRequest() { return new Request("command", 2L); }
      @Override public boolean accepts(Request request) { return request.generation() == 2L && request.prompt().equals("command"); }
      @Override public boolean consumeInferenceResetRequested() { return false; }
      @Override public void recordActionsReceived(int count) { }
      @Override public void discardAcceptedActionChunk() { }
      @Override public void processActionChunk(DoubleBuffer actionChunk, int realActionCount) { }
   }

}
