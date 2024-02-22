package us.ihmc.robotics.geometry;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

/**
 * Utility class for checking if a pose changed since the last time
 * it was checked.
 */
public class FramePose3DChangedTracker
{
   private final FramePose3DReadOnly poseToTrack;
   private FramePose3D lastValue = null;
   private final double tolerance;

   public FramePose3DChangedTracker(FramePose3DReadOnly poseToTrack)
   {
      this(poseToTrack, 0.0001);
   }

   public FramePose3DChangedTracker(FramePose3DReadOnly poseToTrack, double tolerance)
   {
      this.poseToTrack = poseToTrack;
      this.tolerance = tolerance;
   }

   /**
    * @return If the pose has changed since the last time this was called
    *           or true if it has not been called yet or if {@link #markAsChanged()}
    *           was called since this was called.
    */
   public boolean hasChanged()
   {
      if (lastValue == null)
      {
         lastValue = new FramePose3D(poseToTrack);
         return true;
      }
      else
      {
         lastValue.changeFrame(poseToTrack.getReferenceFrame());
         boolean hasChanged = !poseToTrack.geometricallyEquals(lastValue, tolerance);
         lastValue.setIncludingFrame(poseToTrack);
         return hasChanged;
      }
   }

   /**
    * After this is called, the next time {@link #hasChanged()} is called it will return true.
    */
   public void markAsChanged()
   {
      lastValue = null;
   }
}
