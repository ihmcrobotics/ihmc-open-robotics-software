package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.sensorProcessing.heightMap.HeightMapFilterParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface HeightMapFilterParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * If a single cell is higher than all its neighbors by this amount, that cell is
    * labeled an outlier and removed.
    */
   default double getOutlierCellHeightResetEpsilon()
   {
      return get(outlierCellHeightResetEpsilon);
   }

   /**
    * The number of occupied neighbor cells to allow determining if a cell is an
    * outlier.
    */
   default int getMinNeighborsToDetermineOutliers()
   {
      return get(minNeighborsToDetermineOutliers);
   }

   /**
    * Min neighboring cells at the same height to determine that a cell is not an
    * outlier.
    */
   default int getMinNeighborsAtSameHeightForValid()
   {
      return get(minNeighborsAtSameHeightForValid);
   }

   /**
    * Number of cells in a direction to search for data to fill in holes.
    */
   default int getHoleProximityThreshold()
   {
      return get(holeProximityThreshold);
   }

   default boolean getEstimateGroundHeight()
   {
      return get(estimateGroundHeight);
   }

   default boolean getFillHoles()
   {
      return get(fillHoles);
   }

   default boolean getRemoveOutlierCells()
   {
      return get(removeOutlierCells);
   }
}
