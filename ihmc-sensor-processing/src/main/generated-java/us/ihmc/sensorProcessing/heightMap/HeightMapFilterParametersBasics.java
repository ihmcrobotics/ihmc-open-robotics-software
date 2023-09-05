package us.ihmc.sensorProcessing.heightMap;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface HeightMapFilterParametersBasics extends HeightMapFilterParametersReadOnly, StoredPropertySetBasics
{
   /**
    * If a single cell is higher than all its neighbors by this amount, that cell is
    * labeled an outlier and removed.
    */
   default void setOutlierCellHeightResetEpsilon(double outlierCellHeightResetEpsilon)
   {
      set(HeightMapFilterParameters.outlierCellHeightResetEpsilon, outlierCellHeightResetEpsilon);
   }

   /**
    * The number of occupied neighbor cells to allow determining if a cell is an
    * outlier.
    */
   default void setMinNeighborsToDetermineOutliers(int minNeighborsToDetermineOutliers)
   {
      set(HeightMapFilterParameters.minNeighborsToDetermineOutliers, minNeighborsToDetermineOutliers);
   }

   /**
    * Min neighboring cells at the same height to determine that a cell is not an
    * outlier.
    */
   default void setMinNeighborsAtSameHeightForValid(int minNeighborsAtSameHeightForValid)
   {
      set(HeightMapFilterParameters.minNeighborsAtSameHeightForValid, minNeighborsAtSameHeightForValid);
   }

   /**
    * Number of cells in a direction to search for data to fill in holes.
    */
   default void setHoleProximityThreshold(int holeProximityThreshold)
   {
      set(HeightMapFilterParameters.holeProximityThreshold, holeProximityThreshold);
   }

   default void setEstimateGroundHeight(boolean estimateGroundHeight)
   {
      set(HeightMapFilterParameters.estimateGroundHeight, estimateGroundHeight);
   }

   default void setFillHoles(boolean fillHoles)
   {
      set(HeightMapFilterParameters.fillHoles, fillHoles);
   }

   default void setRemoveOutlierCells(boolean removeOutlierCells)
   {
      set(HeightMapFilterParameters.removeOutlierCells, removeOutlierCells);
   }
}
