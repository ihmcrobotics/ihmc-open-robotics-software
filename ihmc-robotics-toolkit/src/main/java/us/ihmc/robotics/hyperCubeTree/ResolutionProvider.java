package us.ihmc.robotics.hyperCubeTree;

public interface ResolutionProvider
{
   public double getResolution(double[] location);
   public double getMinResolution();
}
