package us.ihmc.robotics.hyperCubeTree;

public class ConstantResolutionProvider implements ResolutionProvider
{
   private final double constantResolution;
   public ConstantResolutionProvider(double constantResolution)
   {
      this.constantResolution=constantResolution;
   }
   public double getResolution(double[] location)
   {
      return this.constantResolution;
   }
   public double getMinResolution()
   {
      return constantResolution;
   }
   
}
