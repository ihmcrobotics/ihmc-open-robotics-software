package us.ihmc.sensorProcessing.heightMap;

import com.google.common.util.concurrent.AtomicDouble;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.providers.LongProvider;

/**
 * Contains data for a given XY grid cell.
 */
class HeightMapCell
{
   /* Option to simply take highest point at each cell */
   private static final boolean QUICK_UPDATE = false;

   /** Observed heights within cell */
   private final TDoubleArrayList heightMeasurements = new TDoubleArrayList();
   private final HeightMapParameters parameters;

   private int oldestIndex;
   private final AtomicDouble estimatedHeight = new AtomicDouble();

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
         double lowerBound = estimatedHeight.get() - parameters.getMahalanobisScale() * parameters.getNominalStandardDeviation();
         double upperBound = estimatedHeight.get() + parameters.getMahalanobisScale() * parameters.getNominalStandardDeviation();

         if (heightMeasurements.isEmpty() || height > upperBound)
         {
            // Reset, point is above height threshold to merge
            clear();
            estimatedHeight.set(height);
            heightMeasurements.add(height);
         }
         else if (height < lowerBound)
         {
            // Ignore, point is below height threshold to consider
         }
         else if (heightMeasurements.size() >= parameters.getMaxPointsPerCell())
         {
            // Replace oldest point
            heightMeasurements.set(oldestIndex, height);
            oldestIndex = (oldestIndex + 1) % parameters.getMaxPointsPerCell();
            updateHeightEstimate();
         }
         else
         {
            // Merge with height estimate
            heightMeasurements.add(height);
            updateHeightEstimate();
         }
      }
   }

   private void updateHeightEstimate()
   {
      estimatedHeight.set(heightMeasurements.sum() / heightMeasurements.size());
   }

   void clear()
   {
      heightMeasurements.clear();
      oldestIndex = 0;
      estimatedHeight.set(Double.NaN);
   }

   void resetAtHeight(double height)
   {
      clear();
      estimatedHeight.set(height);
      heightMeasurements.add(height);
   }

   public double getEstimatedHeight()
   {
      return estimatedHeight.get();
   }

   public double computeHeightVariance()
   {
      if (heightMeasurements.isEmpty())
      {
         return Double.NaN;
      }

      double average = 0.0;
      for (int i = 0; i < heightMeasurements.size(); i++)
      {
         average += heightMeasurements.get(i);
      }

      average /= heightMeasurements.size();

      double sigma = 0.0;
      for (int i = 0; i < heightMeasurements.size(); i++)
      {
         sigma += MathTools.square(heightMeasurements.get(i) - average);
      }

      return sigma / heightMeasurements.size();
   }
}
