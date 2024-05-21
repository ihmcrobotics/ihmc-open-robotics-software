package us.ihmc.rdx.ui.vr;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXVRPrescientFootstepStreaming extends RDXVRFootstepPlacement
{
   private static final double STEP_THRESHOLD = 0.01; // 1 cm movement threshold
   private static final double LIFT_THRESHOLD = 0.01; // 1 cm lift threshold
   private static final double STRIDE_LENGTH = 0.75; // fixed stride length in meters
   private final SideDependentList<ReferenceFrame> ankleTrackerFrames = new SideDependentList<>();
   private final SideDependentList<Point3D> initialTrackerPositions = new SideDependentList<>();

   public RDXVRPrescientFootstepStreaming(RDXVRContext vrContext)
   {
      super(vrContext);
   }

   @Override
   public void processVRInput()
   {
      for (RobotSide side : RobotSide.values())
      {
         ReferenceFrame trackerFrame = ankleTrackerFrames.get(side);
         if (trackerFrame != null)
         {
            Point3D currentTrackerPosition = new Point3D();
            trackerFrame.getTransformToWorldFrame().transform(currentTrackerPosition);
            Point3D initialTrackerPosition = initialTrackerPositions.get(side);

            if (initialTrackerPosition == null)
            {
               initialTrackerPositions.put(side, new Point3D(currentTrackerPosition));
               continue;
            }

            Vector3D translation = new Vector3D();
            translation.sub(currentTrackerPosition, initialTrackerPosition);

            // Check if the tracker has moved in any direction
            if (translation.length() >= STEP_THRESHOLD)
            {
               // Check if the foot has been lifted at least 1 cm
               if (currentTrackerPosition.getZ() - initialTrackerPosition.getZ() >= LIFT_THRESHOLD)
               {
                  // Get the current robot foot position in world
                  Tuple3DReadOnly currentRobotFootPosition = syncedRobot.getReferenceFrames().getSoleFrame(side).getTransformToWorldFrame().getTranslation();
                  FramePoint2D currentRobotFootXY = new FramePoint2D(ReferenceFrame.getWorldFrame(), currentRobotFootPosition.getX(), currentRobotFootPosition.getY());
                  // Estimate the final footstep location in the XY plane (world frame)
                  FramePoint2D initialXY = new FramePoint2D(ReferenceFrame.getWorldFrame(), currentRobotFootXY.getX(), currentRobotFootXY.getY());
                  FrameVector2D translationXY = new FrameVector2D(ReferenceFrame.getWorldFrame(), translation.getX(), translation.getY());

                  // Normalize the translation direction to have a fixed stride distance
                  translationXY.normalize();
                  // Scale the normalized direction to the fixed stride distance
                  translationXY.scale(STRIDE_LENGTH);

                  FramePoint2D finalFootstep = new FramePoint2D(initialXY);
                  finalFootstep.add(translationXY);

                  placedFootsteps.add(new RDXVRHandPlacedFootstep(side, footModels.get(side), footstepIndex++, new RigidBodyTransform()));
               }

               // Update the initial position for the next iteration
               initialTrackerPositions.put(side, new Point3D(currentTrackerPosition));
            }
         }
      }
   }

   public void setTrackerReference(RobotSide side, ReferenceFrame trackerReferenceFrame)
   {
      ankleTrackerFrames.put(side, trackerReferenceFrame);
   }
}
