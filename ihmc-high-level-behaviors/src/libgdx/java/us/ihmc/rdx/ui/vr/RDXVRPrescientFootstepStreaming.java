package us.ihmc.rdx.ui.vr;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
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
   private static final double STEP_THRESHOLD = 0.01; // 1 cm movement threshold
   private static final double LIFT_THRESHOLD = 0.01; // 1 cm lift threshold
   private static final double STRIDE_LENGTH = 0.75; // fixed stride length in meters
   private static final double STABILITY_THRESHOLD = 0.005; // 50 mm stability threshold
   private static final int STABILITY_ITERATIONS = 10; // Number of stable iterations

   private final ROS2SyncedRobotModel syncedRobot;
   private final RDXManualFootstepPlacement footstepPlacer;
   private final SideDependentList<ReferenceFrame> ankleTrackerFrames = new SideDependentList<>();
   private final SideDependentList<Boolean> isUserStepping = new SideDependentList<>();
   private final SideDependentList<Point3D> initialTrackerPositions = new SideDependentList<>();
   private final SideDependentList<Point3D> previousTrackerPositions = new SideDependentList<>();
   private final SideDependentList<Integer> stableIterationCounts = new SideDependentList<>();

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
         previousTrackerPositions.put(side, new Point3D());
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
            Point3D currentTrackerPosition = new Point3D();
            trackerFrame.getTransformToWorldFrame().transform(currentTrackerPosition);
            Point3D initialTrackerPosition = initialTrackerPositions.get(side);

            if (!isUserStepping.get(side))
            {
               if (initialTrackerPosition == null)
               {
                  initialTrackerPositions.put(side, new Point3D(currentTrackerPosition));
                  previousTrackerPositions.put(side, new Point3D(currentTrackerPosition));
                  continue;
               }

               Vector3D translation = new Vector3D();
               translation.sub(currentTrackerPosition, initialTrackerPosition);

               // Check if the tracker has moved in any direction
               if (translation.length() >= STEP_THRESHOLD)
               {
                  // Check if the foot has been lifted
                  if (currentTrackerPosition.getZ() - initialTrackerPosition.getZ() >= LIFT_THRESHOLD)
                  {
                     // Get the current robot foot position in world
                     RigidBodyTransform currentRobotFootTransformInWorld = syncedRobot.getReferenceFrames().getSoleFrame(side).getTransformToWorldFrame();
                     FramePoint2D currentRobotFootXY = new FramePoint2D(ReferenceFrame.getWorldFrame(), currentRobotFootTransformInWorld.getTranslation().getX(),
                                                                        currentRobotFootTransformInWorld.getTranslation().getY());
                     // Estimate the final footstep location in the XY plane (world frame)
                     FramePoint2D initialXY = new FramePoint2D(ReferenceFrame.getWorldFrame(), currentRobotFootXY.getX(), currentRobotFootXY.getY());
                     FrameVector2D translationXY = new FrameVector2D(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY());

                     // Normalize the translation direction to have a fixed stride distance
                     translationXY.normalize();
                     // Scale the normalized direction to the fixed stride distance
                     translationXY.scale(STRIDE_LENGTH);
                     // Apply the translation to the current robot foot position
                     FramePoint2D finalFootstep = new FramePoint2D(initialXY);
                     finalFootstep.add(translationXY);

                     currentRobotFootTransformInWorld.getTranslation().setX(finalFootstep.getX());
                     currentRobotFootTransformInWorld.getTranslation().setY(finalFootstep.getX());

                     // Place and send footstep
                     footstepPlacer.createNewFootstep(side);
                     footstepPlacer.setFootstepPose(new FramePose3D(ReferenceFrame.getWorldFrame(), currentRobotFootTransformInWorld));
                     footstepPlacer.placeFootstep();
                     footstepPlacer.exitPlacement();
                     footstepPlacer.walkFromSteps();
                     isUserStepping.put(side, true);
                  }
               }
            }
            else
            {
               if (initialTrackerPosition != null)
               {
                  // Check if tracker is back at initial height
                  double zDifference = Math.abs(currentTrackerPosition.getZ() - initialTrackerPosition.getZ());
                  if (zDifference <= LIFT_THRESHOLD)
                  {
                     // Check if tracker is not moving much anymore
                     Vector3D translation = new Vector3D();
                     translation.sub(currentTrackerPosition, previousTrackerPositions.get(side));

                     if (translation.length() <= STABILITY_THRESHOLD)
                     {
                        int stableCount = stableIterationCounts.get(side);
                        stableCount++;
                        stableIterationCounts.put(side, stableCount);

                        if (stableCount >= STABILITY_ITERATIONS)
                        {
                           isUserStepping.put(side, false);
                           initialTrackerPositions.put(side, new Point3D(currentTrackerPosition));
                           stableIterationCounts.put(side, 0);
                        }
                     }
                     else
                     {
                        stableIterationCounts.put(side, 0);
                     }

                     // Update the previous tracker position for the next iteration
                     previousTrackerPositions.put(side, new Point3D(currentTrackerPosition));
                  }
               }
            }
         }
      }
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
}
