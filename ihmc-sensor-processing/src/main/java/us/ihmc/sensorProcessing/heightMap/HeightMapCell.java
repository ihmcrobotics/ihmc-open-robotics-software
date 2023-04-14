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
   private final AtomicDouble mapHeight = new AtomicDouble();
   private final AtomicDouble mapVariance = new AtomicDouble();

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
         double error = Math.abs(heightMeasurement - estimatedHeight.get());
         double maxErrorForReset = parameters.getMahalanobisScale() * parameters.getNominalStandardDeviation();

         if (heightMeasurements.isEmpty() || error > maxErrorForReset)
         {
            // Reset, point is above height threshold to merge
            clear();
            estimatedHeight.set(heightMeasurement);
            heightMeasurements.add(heightMeasurement);
            varianceMeasurements.add(varianceMeasurement);
         }
         else if (heightMeasurements.size() >= parameters.getMaxPointsPerCell())
         {
            addOldestPointToMapEstimate();
            // Replace oldest point
            heightMeasurements.set(oldestIndex, heightMeasurement);
            varianceMeasurements.set(oldestIndex, varianceMeasurement);
            oldestIndex = (oldestIndex + 1) % parameters.getMaxPointsPerCell();
         }
         else
         {
            // Merge with height estimate
            heightMeasurements.add(heightMeasurement);
            varianceMeasurements.add(varianceMeasurement);
         }
      }
   }

   public void addVariance(double varianceToAdd)
   {
      for (int i = 0; i < varianceMeasurements.size(); i++)
         varianceMeasurements.set(i, varianceMeasurements.get(i) + varianceToAdd);
   }

   private void addOldestPointToMapEstimate()
   {
      if (parameters.getEstimateHeightWithKalmanFilter())
      {

         double height = heightMeasurements.get(oldestIndex);
         double variance = varianceMeasurements.get(oldestIndex);
         if (Double.isNaN(mapHeight.get()))
         {
            // map height hasn't been set yet
            mapHeight.set(height);
            mapVariance.set(variance);
         }
         else
         {
            // roll the oldest measurement into the map height
            double newHeight = (variance * mapHeight.get() + mapVariance.get() * height) / (variance + mapVariance.get());
            double newVariance = variance * mapVariance.get() / (variance + mapVariance.get());

            mapHeight.set(newHeight);
            mapVariance.set(newVariance);
         }
      }
      else if (heightMeasurements.size() > 0)
      {
         double height = heightMeasurements.get(oldestIndex);
         double variance = varianceMeasurements.get(oldestIndex);
         mapHeight.set(height);
         mapVariance.set(variance);
      }
   }



   public void updateHeightEstimate()
   {
      if (parameters.getEstimateHeightWithKalmanFilter())
      {
         double heightSum = 0.0;
         double varianceSum = 0.0;
         for (int i = 0; i < heightMeasurements.size(); i++)
         {
            heightSum += heightMeasurements.get(i) / varianceMeasurements.get(i);
            varianceSum += 1.0 / varianceMeasurements.get(i);
         }
         double newHeight = heightSum / varianceSum;
         // FIXME this is likely not the best way to do this
         double newVariance = varianceMeasurements.get(0);
         for (int i = 1; i < varianceMeasurements.size(); i++)
         {
            newVariance = (newVariance * varianceMeasurements.get(i)) / (newVariance + varianceMeasurements.get(i));
         }

         double heightEstimate;
         if (heightMeasurements.size() == parameters.getMaxPointsPerCell() && !Double.isNaN(mapHeight.get()))
            heightEstimate = (mapHeight.get() * newVariance + newHeight * mapVariance.get()) / (newVariance + mapVariance.get());
         else
            heightEstimate = newHeight;

         estimatedHeight.set(heightEstimate);
      }
      else
      {
         if (Double.isNaN(mapHeight.get()))
            estimatedHeight.set(heightMeasurements.sum() / heightMeasurements.size());
         else
            estimatedHeight.set((heightMeasurements.sum() + mapHeight.get()) / (heightMeasurements.size() + 1));
      }
   }

   public void clear()
   {
      heightMeasurements.clear();
      varianceMeasurements.clear();
      oldestIndex = 0;
      estimatedHeight.set(Double.NaN);
      mapHeight.set(Double.NaN);
      mapVariance.set(Double.NaN);
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
