package us.ihmc.simulationconstructionset;

import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.dataBuffer.DataEntry;

public class DataBufferEntry implements DataEntry
{
   private final YoVariable<?> variable;
   private double[] data;

   private boolean inverted = false;
   
   private double min, max;
   private boolean minMaxChanged = true;

   private boolean minMaxStale = true;

   // private double manualMinScaling = 0.0, manualMaxScaling = 1.0;
   private boolean autoScale = true;

   public DataBufferEntry(YoVariable<?> variable, int nPoints)
   {
      this.variable = variable;

      this.min = 0.0;
      this.max = 0.0;

      double[] emptyData = new double[nPoints];
      this.setData(emptyData, nPoints);
      reCalcMinMax();
   }

   @Override
   public void setInverted(boolean inverted)
   {
      this.inverted = inverted;
   }
   
   @Override
   public boolean getInverted()
   {
      return inverted;
   }

   public int getDataLength()
   {
      return data.length;
   }
   
   @Override
   public double[] getData()
   {
      return this.data;
   }

   @Override
   public void enableAutoScale(boolean autoScale)
   {
      this.autoScale = autoScale;
   }

   @Override
   public void setManualScaling(double minScaling, double maxScaling)
   {
      this.variable.setManualScalingMinMax(minScaling, maxScaling);

      // this.manualMinScaling = minScaling;
      // this.manualMaxScaling = maxScaling;
   }

   @Override
   public boolean isAutoScaleEnabled()
   {
      return this.autoScale;
   }

   @Override
   public double getManualMinScaling()
   {
      return variable.getManualScalingMin();
   } // this.manualMinScaling;}

   @Override
   public double getManualMaxScaling()
   {
      return variable.getManualScalingMax();
   } // this.manualMaxScaling;}

   @Override
   public YoVariable<?> getVariable()
   {
      return this.variable;
   }
   
   @Override
   public String getVariableName()
   {
      return variable.getName();
   }

   @Override
   public String getFullVariableNameWithNameSpace()
   {
     return variable.getFullNameWithNameSpace();
   }

   protected void copyValueThrough()
   {
      for (int i = 0; i < data.length; i++)
      {
         data[i] = variable.getValueAsDouble();
      }

      this.reCalcMinMax();
   }

   protected void enlargeBufferSize(int newSize)
   {
      double[] oldData = data;
      int oldNPoints = oldData.length;

      data = new double[newSize];

      for (int i = 0; i < oldNPoints; i++)
      {
         data[i] = oldData[i];
      }

      for (int i = oldNPoints; i < data.length; i++)
      {
         data[i] = oldData[oldNPoints - 1];
      }

      reCalcMinMax();
   }

   /**
    * Crop the data stored for this variable using the two specified endpoints.  Cropping must reduce or maintain the size of the data set, it cannot be increased.
    *
    * @param start Index of the new start point in the current data
    * @param end Index of the new end point in the current data
    * @return Overall length of the new data set.
    */
   protected int cropData(int start, int end)
   {
      // If the endpoints are unreasonable indicate failure
      if ((start < 0) || (end > data.length))
         return -1;
      
      // Create a temporary variable to hold the old data
      double[] oldData = data;
      int oldNPoints = oldData.length;

      // Calculate the total number of points after the crop
      int nPoints = computeBufferSizeAfterCrop(start, end, oldNPoints);

      // If the result is 0 the size will remain the same
      if (nPoints == 0)
         nPoints = oldNPoints;
      data = new double[nPoints];

      // Transfer the data into the new array beginning with start.
      for (int i = 0; i < data.length; i++)
      {
         data[i] = oldData[(i + start) % oldNPoints];
      }

      // Calculate the Min and Max values for the new set
      reCalcMinMax();

      // Indicate the data length
      return data.length;
   }
   
   
   public int cutData(int start, int end)
   {
      if (start > end) return -1;
      
      // If the endpoints are unreasonable indicate failure
      if ((start < 0) || (end > data.length))
         return -1;
      
      // Create a temporary variable to hold the old data
      double[] oldData = data;
      int oldNPoints = oldData.length;

      // Calculate the total number of points after the cut
      int nPoints = computeBufferSizeAfterCut(start, end, oldNPoints);

      // If the result is 0 the size will remain the same
      if (nPoints == 0)
         nPoints = oldNPoints;
      data = new double[nPoints];

      // Transfer the data into the new array beginning with start.
      int difference = end - start + 1;
      for (int i = 0; i < start; i++)
      {
         data[i] = oldData[i];
      }
      
      for (int i = end + 1; i < oldData.length; i++)
      {
         data[i-difference] = oldData[i];
      }

      // Calculate the Min and Max values for the new set
      reCalcMinMax();

      // Indicate the data length
      return data.length;
   }
   

   public int thinData(int keepEveryNthPoint)
   {
      double[] oldData = data;
      int oldNPoints = oldData.length;
      
      int newNumberOfPoints = oldNPoints / keepEveryNthPoint;
      data = new double[newNumberOfPoints];
      
      int oldDataIndex = 0;
      for (int index = 0; index<newNumberOfPoints; index++)
      {
         data[index] = oldData[oldDataIndex];
         
         oldDataIndex = oldDataIndex + keepEveryNthPoint;
      }
      
      return newNumberOfPoints;
   }

   protected static int computeBufferSizeAfterCrop(int start, int end, int previousBufferSize)
   {
      return ((end - start + 1 + previousBufferSize) % previousBufferSize); 
   }
   
   protected static int computeBufferSizeAfterCut(int start, int end, int previousBufferSize)
   {
      return (previousBufferSize - (end - start + 1)); 
   }
   
   /**
    * Packs the data based on a new start point.  Data is shifted in the array such that the index start is the beginning.  Once the data is shifted the min and max values are relocated.
    *
    * @param start Index of the data point to become the beginning.
    */
   protected void packData(int start)
   {
      // If the start point is outside of the data set abort
      if ((start <= 0) || (start >= data.length))
         return;

      // Create a temporary array to carry out the shift
      double[] oldData = data;
      int nPoints = data.length;
      data = new double[nPoints];

      // Repopulate the array using the new order
      for (int i = 0; i < nPoints; i++)
      {
         data[i] = oldData[(i + start) % nPoints];
      }

      // Recalculate the new min and max values
      reCalcMinMax();
   }

   protected double getVariableValueAsADouble()
   {
      return variable.getValueAsDouble();
   }

   public synchronized void setDataAtIndexToYoVariableValue(int index)
   {
      double newVal = variable.getValueAsDouble();
      double oldVal = data[index];

      data[index] = newVal;

      if (newVal < this.min)
      {
         this.min = newVal;
         setMinMaxChanged();
      }

      if (newVal > this.max)
      {
         this.max = newVal;
         setMinMaxChanged();
      }

      if (oldVal >= this.max)
      {
         setMinMaxChanged();
         minMaxStale = true;
      } // reCalcMinMax();

      if (oldVal <= this.min)
      {
         setMinMaxChanged();
         minMaxStale = true;
      } // reCalcMinMax();
   }

   protected void setYoVariableValueToDataAtIndex(int index)
   {
      double doubleValue = data[index];
      variable.setValueFromDouble(doubleValue);
   }

   @Override
   public synchronized void resetMinMaxChanged()

   // public void resetMinMaxChanged()
   {
      minMaxChanged = false;
   }

   private synchronized void setMinMaxChanged()

   // private void setMinMaxChanged()
   {
      minMaxChanged = true;
   }

   @Override
   public synchronized boolean minMaxChanged()

   // public boolean minMaxChanged()
   {
      return minMaxChanged;
   }

   // private boolean reCalcMinMax()
   private synchronized boolean reCalcMinMax()
   {
      boolean ret = false;

      if (data == null)
         return false;

      minMaxChanged = true;

      double newMin = Double.POSITIVE_INFINITY; // data[0];
      double newMax = Double.NEGATIVE_INFINITY; // data[0];

      for (int i = 1; i < data.length; i++)
      {
         if (!Double.isNaN(data[i]) && data[i] < newMin)
            newMin = data[i];
         if (!Double.isNaN(data[i]) && data[i] > newMax)
            newMax = data[i];
      }
      
      if (newMin > newMax)
      {
         newMin = 0.0;
         newMax = 0.0;
      }

      if ((min != newMin) || (max != newMax))
         ret = true;

      min = newMin;
      max = newMax;

      minMaxStale = false;

      return ret;
   }

   @Override
   public double getMax()
   {
      if (minMaxStale)
         reCalcMinMax();

      return this.max;
   }

   @Override
   public double getMin()
   {
      if (minMaxStale)
         reCalcMinMax();

      return this.min;
   }

   protected void setData(double[] data, int nPoints)
   {
      this.data = data;

      // this.nPoints = nPoints;

      if (data.length != nPoints)
         System.err.println("data and nPoints are not consistent in DataBufferEntry.setData()!!");
      reCalcMinMax();
   }

   protected void setData(double data, int index)
   {
      this.data[index] = data;
      if (data > max)
      {
         max = data;
         setMinMaxChanged();
      }
      if (data < min)
      {
         min = data;
         setMinMaxChanged();
      }
   }
   
   public double computeAverage()
   {
      double total = 0.0;

      int length = data.length;
      for (int i=0; i<length; i++)
      {
         total = total + data[i];
      }

      return total / ((double) length);
   }

   protected double[] getWindowedData(int in, /* int out, */int bufferLength)
   {
      double[] ret = new double[bufferLength];
      int n = in;

      for (int i = 0; i < bufferLength; i++)
      {
         ret[i] = data[n];
         n++;
         if (n >= data.length)
            n = 0;
      }

      return ret;
   }

   @Override
   public double getMax(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex)
   {
      reCalcMinMaxforSetPoint(leftIndex, rightIndex, leftPlotIndex, rightPlotIndex);

      return this.max;
   }

   @Override
   public double getMin(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex)
   {
      reCalcMinMaxforSetPoint(leftIndex, rightIndex, leftPlotIndex, rightPlotIndex);

      return this.min;
   }

   private synchronized boolean reCalcMinMaxforSetPoint(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex)
   {
      // Left Index is the Set IN Point
      // Right Index is the Set OUT Point

      boolean ret = false;

      if (data == null)
      {
         return false;
      }

      minMaxChanged = true;

      double newMin = Double.POSITIVE_INFINITY; // data[leftIndex];
      double newMax = Double.NEGATIVE_INFINITY; // data[leftIndex];
      if (leftIndex < rightIndex)
      {
         for (int i = leftIndex; i < rightIndex; i++)
         {
            if (!Double.isNaN(data[i]) && data[i] < newMin)
               newMin = data[i];
            if (!Double.isNaN(data[i]) && data[i] > newMax)
               newMax = data[i];
         }
      } else
      {
         for (int i = leftIndex; i < rightPlotIndex; i++)
         {
            if (!Double.isNaN(data[i]) && data[i] < newMin)
               newMin = data[i];
            if (!Double.isNaN(data[i]) && data[i] > newMax)
               newMax = data[i];
         }

         for (int i = leftPlotIndex; i < rightIndex; i++)
         {
            if (!Double.isNaN(data[i]) && data[i] < newMin)
               newMin = data[i];
            if (!Double.isNaN(data[i]) && data[i] > newMax)
               newMax = data[i];
         }
      }

      if (newMin > newMax)
      {
         newMin = 0.0;
         newMax = 0.0;
      }

      if ((min != newMin) || (max != newMax))
      {
         ret = true;
      }

      min = newMin;
      max = newMax;

      return ret;
   }

   public boolean checkIfDataIsEqual(DataBufferEntry entry2, int inPoint, int outPoint, double epsilon)
   {
      //      System.out.println(this.variable.getName() + ": InPoint = " + inPoint + ", outPoint = " + outPoint);

      if (inPoint >= this.data.length)
         return false;
      if (inPoint >= entry2.data.length)
         return false;
      if (outPoint >= this.data.length)
         return false;
      if (outPoint >= entry2.data.length)
         return false;

      if (inPoint > outPoint)
         throw new RuntimeException("Sorry, but we assume that inPoint is not greater than outPoint in this method!");

      boolean ret = true;
      for (int i = inPoint; i < outPoint; i++)
      {
         double dataOne = this.data[i];
         double dataTwo = entry2.data[i];

         //         System.out.println(this.variable.getName() + ": dataOne = " + dataOne + ", dataTwo = " + dataTwo);

         if (Math.abs(dataOne - dataTwo) > epsilon)
         {
            ret = false;
         }
      }

      return ret;
   }

   @Override
   public void getVariableNameAndValue(StringBuffer stringBuffer)
   {
      variable.getNameAndValueString(stringBuffer);
   }

   @Override
   public void getVariableNameAndValueAtIndex(StringBuffer stringBuffer, int index)
   {
      variable.getNameAndValueStringFromDouble(stringBuffer, data[index]);
   }

   
}
