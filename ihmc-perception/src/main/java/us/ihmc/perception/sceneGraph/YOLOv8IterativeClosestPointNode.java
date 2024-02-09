package us.ihmc.perception.sceneGraph;

public class YOLOv8IterativeClosestPointNode extends DetectableSceneNode
{
   private int maskErosionKernelRadius;
   private double outlierFilterThreshold;
   private int icpIterations;
   private double baseDistanceThreshold;
   private boolean runICP;

   private double movementDistanceThreshold;

   public YOLOv8IterativeClosestPointNode(long id,
                                          String name,
                                          int maskErosionKernelRadius,
                                          double outlierFilterThreshold,
                                          int icpIterations,
                                          double baseDistanceThreshold,
                                          boolean runICP,
                                          double movementDistanceThreshold)
   {
      super(id, name);

      this.maskErosionKernelRadius = maskErosionKernelRadius;
      this.outlierFilterThreshold = outlierFilterThreshold;
      this.icpIterations = icpIterations;
      this.baseDistanceThreshold = baseDistanceThreshold;
      this.runICP = runICP;
      this.movementDistanceThreshold = movementDistanceThreshold;
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
}
