package us.ihmc.rdx.ui.vr;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.ui.affordances.RDXManualFootstepPlacement;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Class responsible for streaming footstep placements based on VR tracker data.
 * It monitors the ankle tracker positions to predict and place footsteps as the user walks.
 */
public class RDXVRPrescientFootstepStreaming
{
   private static final double STEP_THRESHOLD = 0.06; // movement threshold
   private static final double LIFT_THRESHOLD = 0.02; // lift threshold
   private static final double STRIDE_LENGTH = 0.20; // fixed stride length
   private static final double STABILITY_THRESHOLD = 0.005; // stability threshold
   private static final int STABILITY_ITERATIONS = 20; // Number of stable iterations
   private static final int TURNING_THRESHOLD = 10; // Degrees variation for ankle tracker to trigger 33.3 deg turn on robot
   public static final int WAIT_TIME_BEFORE_STEP = 50; // [ms] time to wait before walking, need some time for the stop streaming status to get to the controller
   public static final int WAIT_TIME_AFTER_STEP = 1000; // [ms] time to wait before restarting streaming

   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXManualFootstepPlacement footstepPlacer;
   private final SideDependentList<ReferenceFrame> ankleTrackerFrames = new SideDependentList<>();
   private final SideDependentList<Boolean> isUserStepping = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> initialTrackersTransform = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> previousTrackersTransform = new SideDependentList<>();
   private final SideDependentList<Integer> stableIterationCounts = new SideDependentList<>();
   private final Notification readyToStep = new Notification();

   /**
    * Constructor for the footstep streaming class.
    *
    * @param syncedRobot the synchronized robot model
    * @param footstepPlacer the footstep placer for manual footstep placement
    */
   public RDXVRPrescientFootstepStreaming(ROS2SyncedRobotModel syncedRobot, RDXManualFootstepPlacement footstepPlacer)
   {
      this.syncedRobot = syncedRobot;
      this.footstepPlacer = footstepPlacer;
      for (RobotSide side : RobotSide.values())
      {
         isUserStepping.put(side, false);
         stableIterationCounts.put(side, 0);
         previousTrackersTransform.put(side, new RigidBodyTransform());
      }
   }

   /**
    * Predicts and places footsteps based on the movement of ankle trackers.
    */
   public void streamFootsteps()
   {
      for (RobotSide side : RobotSide.values())
      {
         ReferenceFrame trackerFrame = ankleTrackerFrames.get(side);
         if (trackerFrame != null)
         {
            RigidBodyTransform currentTrackerTransform = new RigidBodyTransform();
            trackerFrame.getTransformToWorldFrame().transform(currentTrackerTransform);
            RigidBodyTransform initialTrackerTransform = initialTrackersTransform.get(side);

            if (!isUserStepping.get(side))
            {
               if (initialTrackerTransform == null)
               {
                  initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                  previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                  continue;
               }

               Vector3D translation = new Vector3D();
               translation.sub(currentTrackerTransform.getTranslation(), initialTrackerTransform.getTranslation());
               FrameVector2D translationXY = new FrameVector2D(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY());

               // Check if the tracker has moved in any direction
               if (translationXY.norm() >= STEP_THRESHOLD)
               {
                  // Check if the foot has been lifted
                  if (translation.getZ() >= LIFT_THRESHOLD)
                  {
                     // Get the current robot foot position in world
                     RigidBodyTransform currentRobotFootTransformInWorld = new RigidBodyTransform(syncedRobot.getReferenceFrames().getSoleFrame(side).getTransformToWorldFrame());
                     FramePoint2D currentRobotFootXY = new FramePoint2D(ReferenceFrame.getWorldFrame(), currentRobotFootTransformInWorld.getTranslation().getX(),
                                                                        currentRobotFootTransformInWorld.getTranslation().getY());
                     // Estimate the final footstep location in the XY plane (world frame)
                     FramePoint2D initialXY = new FramePoint2D(ReferenceFrame.getWorldFrame(), currentRobotFootXY.getX(), currentRobotFootXY.getY());

                     // Normalize the translation direction to have a fixed stride distance
                     translationXY.normalize();
                     // Scale the normalized direction to the fixed stride distance
                     translationXY.scale(STRIDE_LENGTH);
                     // Apply the translation to the current robot foot position
                     FramePoint2D finalFootstep = new FramePoint2D(initialXY);
                     finalFootstep.add(translationXY);
                     currentRobotFootTransformInWorld.getTranslation().setX(finalFootstep.getX());
                     currentRobotFootTransformInWorld.getTranslation().setY(finalFootstep.getY());
                     // Compute yaw variation
                     double newYaw = currentRobotFootTransformInWorld.getRotation().getYaw();
                     double yawVariation = currentTrackerTransform.getRotation().getYaw() - initialTrackerTransform.getRotation().getYaw();
                     if (Math.toDegrees(yawVariation) >= TURNING_THRESHOLD)
                     {
                        newYaw += Math.toRadians(33.33);
                     }
                     else if (Math.toDegrees(yawVariation) <= -TURNING_THRESHOLD)
                     {
                        newYaw -= Math.toRadians(33.33);
                     }
                     // Update yaw of footstep
                     currentRobotFootTransformInWorld.getRotation().setYawPitchRoll(newYaw,
                                                                                    currentRobotFootTransformInWorld.getRotation().getPitch(),
                                                                                    currentRobotFootTransformInWorld.getRotation().getRoll());

                     // Place and send footstep
                     footstepPlacer.createNewFootstep(side);
                     footstepPlacer.setFootstepPose(new FramePose3D(ReferenceFrame.getWorldFrame(), currentRobotFootTransformInWorld));
                     footstepPlacer.checkAndPlaceFootstep();
                     footstepPlacer.exitPlacement();
                     isUserStepping.put(side, true);
                     readyToStep.set();
                  }
               }
            }
            else
            {
               if (initialTrackerTransform != null)
               {
                  // Check if tracker is not moving much anymore
                  Vector3D translation = new Vector3D();
                  translation.sub(currentTrackerTransform.getTranslation(), previousTrackersTransform.get(side).getTranslation());

                  if (translation.norm() <= STABILITY_THRESHOLD)
                  {
                     int stableCount = stableIterationCounts.get(side);
                     stableCount++;
                     stableIterationCounts.put(side, stableCount);

                     if (stableCount >= STABILITY_ITERATIONS)
                     {
                        isUserStepping.put(side, false);
                        initialTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
                        stableIterationCounts.put(side, 0);
                     }
                  }
                  else
                  {
                     stableIterationCounts.put(side, 0);
                  }

                  // Update the previous tracker position for the next iteration
                  previousTrackersTransform.put(side, new RigidBodyTransform(currentTrackerTransform));
               }
            }
         }
      }
   }

   public Notification getReadyToStepNotification()
   {
      return readyToStep;
   }

   public void step()
   {
      footstepPlacer.walkFromSteps();
   }

   /**
    * Sets the reference frame for the tracker of a given side.
    *
    * @param side the side (left or right) of the robot
    * @param trackerReferenceFrame the reference frame of the tracker
    */
   public void setTrackerReference(RobotSide side, ReferenceFrame trackerReferenceFrame)
   {
      ankleTrackerFrames.put(side, trackerReferenceFrame);
   }

   public void reset()
   {
      for (RobotSide side : RobotSide.values())
      {
         ankleTrackerFrames.put(side, null);
         isUserStepping.put(side, false);
         initialTrackersTransform.put(side, null);
      }
   }
}
