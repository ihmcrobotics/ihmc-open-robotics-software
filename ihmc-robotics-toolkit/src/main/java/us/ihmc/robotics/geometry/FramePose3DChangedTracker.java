package us.ihmc.robotics.geometry;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

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
}
