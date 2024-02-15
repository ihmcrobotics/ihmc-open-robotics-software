package us.ihmc.perception.sceneGraph;

public class YOLOv8IterativeClosestPointNode extends DetectableSceneNode
{
   // Read from RDX node, write to YOLO ICP Combo
   private int maskErosionKernelRadius;
   private double outlierFilterThreshold;
   private int icpIterations;
   private double baseDistanceThreshold;
   private boolean runICP;

   // Read from YOLO ICP Combo, write to RDX node
   private double movementDistanceThreshold;
   private double detectionFrequency;

   public YOLOv8IterativeClosestPointNode(long id,
                                          String name,
                                          int maskErosionKernelRadius,
                                          double outlierFilterThreshold,
                                          int icpIterations,
                                          double baseDistanceThreshold,
                                          boolean runICP,
                                          double movementDistanceThreshold,
                                          double detectionFrequency)
   {
      super(id, name);

      this.maskErosionKernelRadius = maskErosionKernelRadius;
      this.outlierFilterThreshold = outlierFilterThreshold;
      this.icpIterations = icpIterations;
      this.baseDistanceThreshold = baseDistanceThreshold;
      this.runICP = runICP;
      this.movementDistanceThreshold = movementDistanceThreshold;
      this.detectionFrequency = detectionFrequency;
   }

   public int getMaskErosionKernelRadius()
   {
      return maskErosionKernelRadius;
   }

   public void setMaskErosionKernelRadius(int maskErosionKernelRadius)
   {
      this.maskErosionKernelRadius = maskErosionKernelRadius;
   }

   public double getOutlierFilterThreshold()
   {
      return outlierFilterThreshold;
   }

   public void setOutlierFilterThreshold(double outlierFilterThreshold)
   {
      this.outlierFilterThreshold = outlierFilterThreshold;
   }

   public int getICPIterations()
   {
      return icpIterations;
   }

   public void setICPIterations(int icpIterations)
   {
      this.icpIterations = icpIterations;
   }

   public double getBaseDistanceThreshold()
   {
      return baseDistanceThreshold;
   }

   public void setBaseDistanceThreshold(double baseDistanceThreshold)
   {
      this.baseDistanceThreshold = baseDistanceThreshold;
   }

   public boolean isRunningICP()
   {
      return runICP;
   }

   public void setRunICP(boolean runICP)
   {
      this.runICP = runICP;
   }

   public double getMovementDistanceThreshold()
   {
      return movementDistanceThreshold;
   }

   public void setMovementDistanceThreshold(double movementDistanceThreshold)
   {
      this.movementDistanceThreshold = movementDistanceThreshold;
   }

   public double getDetectionFrequency()
   {
      return detectionFrequency;
   }

   public void setDetectionFrequency(double detectionFrequency)
   {
      this.detectionFrequency = detectionFrequency;
   }
}
