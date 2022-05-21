package us.ihmc.perception.gpuHeightMap;

import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLManager;

public class GPUHeightMap
{
   private final GPUHeightMapParameters parameters;
   private final int numberOfCells;

   private final OpenCLManager openCLManager = new OpenCLManager();

   private final BytedecoImage elevationMap;
   private final BytedecoImage varianceMap;
   private final BytedecoImage validityMap;
   private final BytedecoImage traversabilityMap;
   private final BytedecoImage timeMap;
   private final BytedecoImage upperBoundMap;
   private final BytedecoImage lowerBoundMap;

   public GPUHeightMap(GPUHeightMapParameters parameters)
   {
      this.parameters = parameters;

      // the added two are for the borders
      numberOfCells = ((int) Math.round(parameters.mapLength / parameters.resolution)) + 2;
   }
   // todo elevation map is a seven layer array (7 x n x n)
   // todo normal map is a a three layer array (3 x n x n)
}
