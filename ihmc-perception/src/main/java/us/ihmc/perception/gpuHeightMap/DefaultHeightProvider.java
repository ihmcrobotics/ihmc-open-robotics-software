package us.ihmc.perception.gpuHeightMap;

/**
 * Interface used to compute the default height at which to initialize the height map whenever the reset method is called.
 */
public interface DefaultHeightProvider
{
   double computeDefaultHeight();
}
