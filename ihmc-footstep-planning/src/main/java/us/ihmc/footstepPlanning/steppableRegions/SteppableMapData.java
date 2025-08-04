package us.ihmc.footstepPlanning.steppableRegions;

import org.bytedeco.opencv.opencv_core.Mat;

public class SteppableMapData
{

   private Mat steppableRegionAssignmentMat;
   private Mat steppableRegionRingMat;
   private Mat steppabilityConnectionsImage;

   public SteppableMapData(SteppableMapData other)
   {
      setSteppableRegionAssignmentMat(other.steppableRegionAssignmentMat);
      setSteppableRegionRingMat(other.steppableRegionRingMat);
      setSteppabilityConnectionsImage(other.steppabilityConnectionsImage);
   }

   public void setSteppabilityConnectionsImage(Mat steppabilityConnectionsImage)
   {
      this.steppabilityConnectionsImage = steppabilityConnectionsImage == null ? null : steppabilityConnectionsImage.clone();
   }

   public void setSteppableRegionAssignmentMat(Mat steppableRegionAssignmentMat)
   {
      this.steppableRegionAssignmentMat = steppableRegionAssignmentMat == null ? null : steppableRegionAssignmentMat.clone();
   }

   public void setSteppableRegionRingMat(Mat steppableRegionRingMat)
   {
      this.steppableRegionRingMat = steppableRegionRingMat == null ? null : steppableRegionRingMat.clone();
   }

   public Mat getSteppableRegionAssignmentMat()
   {
      return steppableRegionAssignmentMat;
   }

   public Mat getSteppableRegionRingMat()
   {
      return steppableRegionRingMat;
   }

   public Mat getSteppabilityConnectionsImage()
   {
      return steppabilityConnectionsImage;
   }
}
