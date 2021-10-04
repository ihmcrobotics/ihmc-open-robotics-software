package us.ihmc.sensorProcessing.heightMap;

import com.google.common.util.concurrent.AtomicDouble;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.providers.LongProvider;

/**
 * Contains data for a given XY grid cell.
 */
class HeightMapCell
{
   private static final boolean QUICK_UPDATE = false;

   /** Observed heights within cell */
   private final TDoubleArrayList heightMeasurements = new TDoubleArrayList();
   private final HeightMapParameters parameters;

   private int oldestIndex;
   private final AtomicDouble estimatedHeight = new AtomicDouble();
   private double variance;

   public HeightMapCell(HeightMapParameters parameters)
   {
      this.parameters = parameters;
      clear();
   }

   void addPoint(double height)
   {
      if (QUICK_UPDATE)
      {
         if (Double.isNaN(estimatedHeight.get()))
         {
            estimatedHeight.set(height);
         }
         else
         {
            estimatedHeight.set(Math.max(estimatedHeight.get(), height));
         }
      }
      else
      {
         if (heightMeasurements.isEmpty() || height > estimatedHeight.get() + parameters.getMahalanobisScale() * variance)
         {
            // Reset, point is above height threshold to merge
            clear();
            estimatedHeight.set(height);
//            variance = parameters.getNominalHeightVariance();
            heightMeasurements.add(height);
         }
         else if (height < estimatedHeight.get() - parameters.getMahalanobisScale() * variance)
         {
            // Ignore, point is below height threshold to consider
         }
         else if (heightMeasurements.size() >= parameters.getMaxPointsPerCell())
         {
            // Replace oldest point
            heightMeasurements.set(oldestIndex, height);
            oldestIndex = (oldestIndex + 1) % parameters.getMaxPointsPerCell();
            updateHeightEstimate(false);
         }
         else
         {
            // Merge with height estimate
            heightMeasurements.add(height);
            updateHeightEstimate(true);
         }
      }
   }

   private void updateHeightEstimate(boolean recomputeVariance)
   {
      double estimatedHeight = 0.0;
      for (int i = 0; i < heightMeasurements.size(); i++)
      {
         estimatedHeight += heightMeasurements.get(i);
      }
      estimatedHeight /= heightMeasurements.size();
      this.estimatedHeight.set(estimatedHeight);

      if (recomputeVariance)
      {
//         variance = parameters.getNominalHeightVariance() / Math.sqrt(heightMeasurements.size());
      }
   }

   void clear()
   {
      heightMeasurements.clear();

      oldestIndex = 0;
      estimatedHeight.set(Double.NaN);
      variance = Double.NaN;
   }

   public double getEstimatedHeight()
   {
      return estimatedHeight.get();
   }
}
