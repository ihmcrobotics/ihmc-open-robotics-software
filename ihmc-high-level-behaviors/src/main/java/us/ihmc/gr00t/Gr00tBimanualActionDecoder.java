package us.ihmc.gr00t;

import us.ihmc.euclid.geometry.Pose3D;

import java.nio.DoubleBuffer;
import java.util.ArrayList;
import java.util.List;

/** Decoder for [left wrist, right wrist, neck pitch/yaw, left hand, right hand] action rows. */
public final class Gr00tBimanualActionDecoder implements Gr00tActionDecoder<Gr00tHumanoidAction>
{
   public static final int WRIST_POSE_SIZE = 7;
   public static final int NECK_SIZE = 2;
   private final int handTargetCount;
   private final int actionSize;

   public Gr00tBimanualActionDecoder(int handTargetCount)
   {
      if (handTargetCount < 0)
         throw new IllegalArgumentException("handTargetCount must be non-negative");
      this.handTargetCount = handTargetCount;
      actionSize = 2 * WRIST_POSE_SIZE + NECK_SIZE + 2 * handTargetCount;
   }

   public int getActionSize()
   {
      return actionSize;
   }

   @Override
   public List<Gr00tHumanoidAction> decode(DoubleBuffer actionChunk, int realActionCount)
   {
      int availableRows = Math.min(Math.max(realActionCount, 0), actionChunk.capacity() / actionSize);
      List<Gr00tHumanoidAction> actions = new ArrayList<>(availableRows);
      for (int row = 0; row < availableRows; row++)
      {
         if (!isValidRow(actionChunk, row))
            continue;

         Pose3D leftPose = poseAtRow(actionChunk, row, 0);
         Pose3D rightPose = poseAtRow(actionChunk, row, WRIST_POSE_SIZE);
         int neckOffset = row * actionSize + 2 * WRIST_POSE_SIZE;
         double neckPitch = actionChunk.get(neckOffset);
         double neckYaw = actionChunk.get(neckOffset + 1);
         double[] leftHand = new double[handTargetCount];
         double[] rightHand = new double[handTargetCount];
         int handOffset = neckOffset + NECK_SIZE;
         for (int target = 0; target < handTargetCount; target++)
         {
            leftHand[target] = actionChunk.get(handOffset + target);
            rightHand[target] = actionChunk.get(handOffset + handTargetCount + target);
         }
         actions.add(new Gr00tHumanoidAction(leftPose, rightPose, neckPitch, neckYaw, leftHand, rightHand));
      }
      return List.copyOf(actions);
   }

   private Pose3D poseAtRow(DoubleBuffer actionChunk, int row, int wristOffset)
   {
      int offset = row * actionSize + wristOffset;
      Pose3D pose = new Pose3D();
      pose.getPosition().set(actionChunk.get(offset), actionChunk.get(offset + 1), actionChunk.get(offset + 2));
      pose.getOrientation().set(actionChunk.get(offset + 3),
                                actionChunk.get(offset + 4),
                                actionChunk.get(offset + 5),
                                actionChunk.get(offset + 6));
      pose.getOrientation().normalize();
      return pose;
   }

   private boolean isValidRow(DoubleBuffer actionChunk, int row)
   {
      int rowOffset = row * actionSize;
      for (int dimension = 0; dimension < actionSize; dimension++)
      {
         if (!Double.isFinite(actionChunk.get(rowOffset + dimension)))
            return false;
      }

      for (int wristOffset : new int[] {0, WRIST_POSE_SIZE})
      {
         double normSquared = 0.0;
         for (int quaternionElement = 0; quaternionElement < 4; quaternionElement++)
         {
            double value = actionChunk.get(rowOffset + wristOffset + 3 + quaternionElement);
            normSquared += value * value;
         }
         if (!(normSquared > 1.0e-12))
            return false;
      }
      return true;
   }
}
