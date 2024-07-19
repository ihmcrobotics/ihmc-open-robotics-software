package us.ihmc.perception.detections.yolo;

import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class YOLOv8Settings extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey confidenceThreshold = keys.addDoubleKey("Confidence Threshold", 0.8);
   public static final DoubleStoredPropertyKey nonMaximumSupressionThreshold = keys.addDoubleKey("Non-Maximum Suppression Threshold", 0.1);
   public static final DoubleStoredPropertyKey maskThreshold = keys.addDoubleKey("Mask Threshold", 0.0f);
   public static final DoubleStoredPropertyKey outlierRejectionThreshold = keys.addDoubleKey("Outlier Rejection Threshold", 1.0f);
   public static final IntegerStoredPropertyKey erosionKernelRadius = keys.addIntegerKey("Erosion Kernel Radius", 2);

   public YOLOv8Settings()
   {
      super(keys, YOLOv8Settings.class);
   }

   public double getConfidenceThreshold()
   {
      return get(confidenceThreshold);
   }

   public double getNMSThreshold()
   {
      return get(nonMaximumSupressionThreshold);
   }

   public double getMaskThreshold()
   {
      return get(maskThreshold);
   }

   public double getOutlierRejectionThreshold()
   {
      return get(outlierRejectionThreshold);
   }

   public int getErosionKernelRadius()
   {
      return get(erosionKernelRadius);
   }

   public void setConfidenceThreshold(double confidenceThreshold)
   {
      set(YOLOv8Settings.confidenceThreshold, confidenceThreshold);
   }

   public void setNMSThreshold(double nmsThreshold)
   {
      set(YOLOv8Settings.nonMaximumSupressionThreshold, nmsThreshold);
   }

   public void setMaskThreshold(double maskThreshold)
   {
      set(YOLOv8Settings.maskThreshold, maskThreshold);
   }

   public void setErosionKernelRadius(int erosionKernelRadius)
   {
      set(YOLOv8Settings.erosionKernelRadius, erosionKernelRadius);
   }
}
