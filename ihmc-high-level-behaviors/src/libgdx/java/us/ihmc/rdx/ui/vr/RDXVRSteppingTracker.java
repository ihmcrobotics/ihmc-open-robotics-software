package us.ihmc.rdx.ui.vr;

import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXVRSteppingTracker
{
    private static final double STEP_THRESHOLD = 0.02;
    private static final double LIFT_THRESHOLD = 0.02;
   private static final double LANDING_THRESHOLD = 0.04;
    private static final double STABILITY_THRESHOLD = 0.01;
    private static final int STABILITY_ITERATIONS = 3;

   private static final double CLOSE_TO_GROUND_HEIGHT_THRESHOLD = 0.1;
   private static final double CLOSE_TO_GROUND_BLEND_START_HEIGHT = 0.25;

   private static final double ERROR_RECOVERY_HEIGHT_TOLERANCE = 0.05; // 5 cm
   private static final int ERROR_RECOVERY_STABILITY_ITERATIONS = 50;
   private final SideDependentList<Integer> errorRecoveryStableCounts = new SideDependentList<>();

    private final SideDependentList<Boolean> isUserStepping = new SideDependentList<>();
    private final SideDependentList<RigidBodyTransform> initialTrackersTransform = new SideDependentList<>();
    private final SideDependentList<RigidBodyTransform> previousTrackersTransform = new SideDependentList<>();
    private final SideDependentList<Integer> stableIterationCounts = new SideDependentList<>();
    private final SideDependentList<Notification> landedFoot = new SideDependentList<>();
   private final SideDependentList<Double> currentFootHeight = new SideDependentList<>();
   private final SideDependentList<Boolean> isSwinging = new SideDependentList<>();

    public RDXVRSteppingTracker()
    {
        reset();
    }

    public void reset()
    {
        for (RobotSide side : RobotSide.values())
        {
            isUserStepping.put(side, false);
            stableIterationCounts.put(side, 0);
            previousTrackersTransform.put(side, new RigidBodyTransform());
            initialTrackersTransform.put(side, null);
            landedFoot.put(side, new Notification());
            errorRecoveryStableCounts.put(side, 0);
            isSwinging.put(side, false);
        }
    }

    public void processVRInput(RobotSide side, RigidBodyTransform currentTrackerTransform)
    {
        RigidBodyTransform initialTrackerTransform = initialTrackersTransform.get(side);
        if (initialTrackerTransform == null)
        {
            initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
            previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
        }

        // Get translation of current tracker position wrt to initial one
        Vector3D translationTracker = new Vector3D();
        translationTracker.sub(currentTrackerTransform.getTranslation(), initialTrackersTransform.get(side).getTranslation());
        FrameVector2D translationTrackerXY = new FrameVector2D(ReferenceFrame.getWorldFrame(),
                translationTracker.getX(),
                translationTracker.getY());
        currentFootHeight.put(side, translationTracker.getZ());

        if (!isUserStepping.get(side)) // Tracker is not moving by a lot yet
        {
            // Check if the tracker has moved in any direction AND the foot has been lifted
            if (translationTrackerXY.norm() >= STEP_THRESHOLD
                    && translationTracker.getZ() >= LIFT_THRESHOLD)
            {
                isUserStepping.put(side, true);
                LogTools.debug("User stepping with {}", side);
                // Avoid false stepping detection when already in swing with one side
                isUserStepping.put(side.getOppositeSide(), false);
            }
        }
        else // Already stepping, tracker is moving
        {
            // Avoid false stepping detection when already in swing with one side
            isUserStepping.put(side.getOppositeSide(), false);
            if (initialTrackerTransform != null)
            {
                // Check if tracker is not moving much anymore
                Vector3D translationFromPreviousPosition = new Vector3D();
                translationFromPreviousPosition.sub(currentTrackerTransform.getTranslation(), previousTrackersTransform.get(side).getTranslation());

                // If the tracker is not moving much, then start counting for stability
                if (translationFromPreviousPosition.norm() <= STABILITY_THRESHOLD && translationTracker.getZ() < LIFT_THRESHOLD)
                {
                    int stableCount = stableIterationCounts.get(side);
                    stableCount++;
                    stableIterationCounts.put(side, stableCount);

                    if (stableCount >= STABILITY_ITERATIONS)
                    {
                        resetSide(side, currentTrackerTransform);
                        LogTools.debug("User completed stepping with {}", side);
                        landedFoot.get(side).set();
                    }
                }
                else  // Still moving
                {
                    stableIterationCounts.put(side, 0); // reset stability count
                }
                // Update the previous tracker position for the next iteration
                previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
            }
        }

       // -------------------------------
       // Error recovery / re-sync logic
       // -------------------------------
       // If the tracker height is close to the initial height (+/- 5 cm)
       // and not moving much overall, count stable iterations and then reset.
       if (initialTrackerTransform != null)
       {
          double heightError = Math.abs(currentTrackerTransform.getTranslationZ() - initialTrackerTransform.getTranslationZ());

          // Compare full 3D translation between current and previous poses
          Vector3D translationFromPreviousPosition = new Vector3D();
          translationFromPreviousPosition.sub(currentTrackerTransform.getTranslation(),
                                              previousTrackersTransform.get(side).getTranslation());

          if (heightError <= ERROR_RECOVERY_HEIGHT_TOLERANCE
              && translationFromPreviousPosition.norm() <= STABILITY_THRESHOLD)
          {
             int count = errorRecoveryStableCounts.get(side);
             count++;
             errorRecoveryStableCounts.put(side, count);

             if (count >= ERROR_RECOVERY_STABILITY_ITERATIONS)
             {
                // Re-sync this tracker as if this pose were the new \"standing\" pose
                resetSide(side, currentTrackerTransform);
                LogTools.debug("Error recovery reset for {}", side);
                // Also clear the error-recovery counter
                errorRecoveryStableCounts.put(side, 0);
             }
          }
          else
          {
             errorRecoveryStableCounts.put(side, 0);
          }
       }
       // -------------------------------
    }

    private void resetSide(RobotSide side, RigidBodyTransform currentTrackerTransform)
    {
        isUserStepping.put(side, false);
        initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
        stableIterationCounts.put(side, 0);
    }

    public boolean isFootInContact(RobotSide side)
    {
        return !isUserStepping.get(side);
    }

   public boolean isFootLanding(RobotSide side)
   {
      if (side == RobotSide.LEFT)
         LogTools.warn(currentFootHeight.get(side));
      // landing = not in contact, was swinging at least once, and now below 5 cm
      if (!isUserStepping.get(side))
         return false;

      if (!isSwinging.get(side))
         return false;

      return currentFootHeight.get(side) <= LANDING_THRESHOLD;
   }

   public void setIsSwinging(RobotSide side, boolean value)
   {
      isSwinging.put(side, value);
   }

   public boolean isFootCloseToGround(RobotSide side)
   {
      if (!isUserStepping.get(side))
         return false;

      double height = currentFootHeight.get(side);
      return height <= CLOSE_TO_GROUND_BLEND_START_HEIGHT;
   }

   public double getCloseToGroundBlendFactor(RobotSide side)
   {
      if (!isFootCloseToGround(side))
         return 0.0;

      double height = currentFootHeight.get(side);
      double start = CLOSE_TO_GROUND_BLEND_START_HEIGHT;
      double end = CLOSE_TO_GROUND_HEIGHT_THRESHOLD;

      if (height >= start)
         return 0.0;
      if (height <= end)
         return 1.0;

      // map height in [start, end] to alpha in [0,1]
      double alpha = (start - height) / (start - end);
      return Math.max(0.0, Math.min(1.0, alpha));
   }

   public Notification getLandedFootNotification(RobotSide side)
    {
        return landedFoot.get(side);
    }
}
