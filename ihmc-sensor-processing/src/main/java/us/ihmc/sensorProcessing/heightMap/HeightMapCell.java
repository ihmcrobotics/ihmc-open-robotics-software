package us.ihmc.sensorProcessing.heightMap;

import com.google.common.util.concurrent.AtomicDouble;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;

/**
 * Contains data for a given XY grid cell.
 */
public class HeightMapCell
{
   /* Option to simply take highest point at each cell */
   private static final boolean QUICK_UPDATE = false;

   /** Observed heights within cell */
   private final TDoubleArrayList heightMeasurements = new TDoubleArrayList();
   private final TDoubleArrayList varianceMeasurements = new TDoubleArrayList();
   private final HeightMapParametersReadOnly parameters;

   private int oldestIndex;
   private final AtomicDouble estimatedHeight = new AtomicDouble();

   /** Filtered parameters */
   private boolean isGroundCell = false;
   private boolean hasSufficientNeighbors = false;

   public HeightMapCell(HeightMapParametersReadOnly parameters)
   {
      this.parameters = parameters;
      clear();
   }

   public void addPoint(double heightMeasurement, double varianceMeasurement)
   {
      if (QUICK_UPDATE)
      {
         if (Double.isNaN(estimatedHeight.get()))
         {
            estimatedHeight.set(heightMeasurement);
         }
         else
         {
            estimatedHeight.set(Math.max(estimatedHeight.get(), heightMeasurement));
         }
      }
      else
      {
         double lowerBound = estimatedHeight.get() - parameters.getMahalanobisScale() * parameters.getNominalStandardDeviation();
         double upperBound = estimatedHeight.get() + parameters.getMahalanobisScale() * parameters.getNominalStandardDeviation();

         if (heightMeasurements.isEmpty() || heightMeasurement > upperBound)
         {
            // Reset, point is above height threshold to merge
            clear();
            estimatedHeight.set(heightMeasurement);
            heightMeasurements.add(heightMeasurement);
            varianceMeasurements.add(varianceMeasurement);
         }
         else if (heightMeasurement < lowerBound)
         {
            // Ignore, point is below height threshold to consider
         }
         else if (heightMeasurements.size() >= parameters.getMaxPointsPerCell())
         {
            // Replace oldest point
            heightMeasurements.set(oldestIndex, heightMeasurement);
            varianceMeasurements.set(oldestIndex, varianceMeasurement);
            oldestIndex = (oldestIndex + 1) % parameters.getMaxPointsPerCell();
            updateHeightEstimate();
         }
         else
         {
            // Merge with height estimate
            heightMeasurements.add(heightMeasurement);
            varianceMeasurements.add(varianceMeasurement);
            updateHeightEstimate();
         }
      }
   }

   private void updateHeightEstimate()
   {
      double heightSum = 0.0;
      double varianceSum = 0.0;
      for (int i = 0; i < heightMeasurements.size(); i++)
      {
         heightSum += heightMeasurements.get(i) / varianceMeasurements.get(i);
         varianceSum += varianceMeasurements.get(i);
      }
      estimatedHeight.set(heightSum / varianceSum);
   }

   public void clear()
   {
      heightMeasurements.clear();
      varianceMeasurements.clear();
      oldestIndex = 0;
      estimatedHeight.set(Double.NaN);
   }

   public void resetAtHeight(double heightMeasurement, double varianceMeasurement)
   {
      clear();
      estimatedHeight.set(heightMeasurement);
      heightMeasurements.add(heightMeasurement);
      varianceMeasurements.add(varianceMeasurement);
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

   public void setGroundCell(boolean isGroundCell)
   {
      this.isGroundCell = isGroundCell;
   }

   public boolean isGroundCell()
   {
      return isGroundCell;
   }

   public void setHasSufficientNeighbors(boolean hasSufficientNeighbors)
   {
      this.hasSufficientNeighbors = hasSufficientNeighbors;
   }

   public boolean hasSufficientNeighbors()
   {
      return hasSufficientNeighbors;
   }
}
