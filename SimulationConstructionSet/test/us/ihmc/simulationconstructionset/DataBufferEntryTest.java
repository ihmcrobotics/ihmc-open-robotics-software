package us.ihmc.simulationconstructionset;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class DataBufferEntryTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVal()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double tempDouble = (double) random.nextInt(20000) / (double) random.nextInt(30);
      doubleYoVariable.set(tempDouble);
      assertEquals(tempDouble, dataBufferEntry.getVariableValueAsADouble(), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTickAndUpdate()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double epsilon = 0;
      double[] tempData = new double[nPoints];
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      double[] data = dataBufferEntry.getData();

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], data[i], epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000) public void testComputeAverage()
   {
      int nPoints = 100;
      
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double total = 0.0;
      for (int i = 0; i < nPoints; i++)
      {
         double data = RandomNumbers.nextDouble(random, -100.0, 100.0);
         doubleYoVariable.set(data);
         total = total + data;
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      double average = total / ((double) nPoints);
      double computedAverage = dataBufferEntry.computeAverage();

      assertEquals(average, computedAverage, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testUpdateValue()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double epsilon = 0;
      double[] tempData = new double[nPoints];
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      for (int i = 0; i < nPoints; i++)
      {
         dataBufferEntry.setYoVariableValueToDataAtIndex(i);
         assertEquals(doubleYoVariable.getValueAsDouble(), dataBufferEntry.getData()[i], epsilon);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCheckIfDataIsEqual()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double temp;
      double epsilon = 0;
      DoubleYoVariable doubleYoVariable2 = new DoubleYoVariable("doubleYoVariable2", null);
      doubleYoVariable2.set(0);
      DataBufferEntry entry2 = new DataBufferEntry(doubleYoVariable2, nPoints);

      for (int i = 0; i < nPoints; i++)
      {
         temp = (double) random.nextInt(20000) / (double) random.nextInt(30);

         doubleYoVariable.set(temp);
         doubleYoVariable2.set(temp);

         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
         entry2.setDataAtIndexToYoVariableValue(i);
      }

      assertTrue(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints - 1, epsilon));

      dataBufferEntry = entry2 = null;

      dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      entry2 = new DataBufferEntry(doubleYoVariable2, nPoints);

      for (int i = 0; i < nPoints; i++)
      {
         temp = (double) random.nextInt(20000) / (double) random.nextInt(30);

         doubleYoVariable.set(temp);
         doubleYoVariable2.set(-temp);

         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
         entry2.setDataAtIndexToYoVariableValue(i);
      }

      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints - 1, epsilon));
      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints, epsilon)); // outPoint out of bounds
      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, nPoints, nPoints - 1, epsilon)); //inPoint out of bounds
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMinAndMaxScaling()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double minScaling = random.nextDouble();
      double maxScaling = random.nextDouble() + 2;
      dataBufferEntry.setManualScaling(minScaling, maxScaling);

      assertEquals(minScaling, dataBufferEntry.getManualMinScaling(), 0);
      assertEquals(maxScaling, dataBufferEntry.getManualMaxScaling(), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetVariable()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      assertEquals(doubleYoVariable, dataBufferEntry.getVariable());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCopyValueThrough()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double tempDouble = random.nextDouble();
      doubleYoVariable.set(tempDouble);
      dataBufferEntry.setDataAtIndexToYoVariableValue(0);
      dataBufferEntry.copyValueThrough();

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempDouble, dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEnlargeBufferSize()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double[] tempData = new double[nPoints];
      int newSizeDelta = random.nextInt(100);

      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      dataBufferEntry.enlargeBufferSize(nPoints + newSizeDelta);

      assertEquals(nPoints + newSizeDelta, dataBufferEntry.getData().length);

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCropData()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double[] tempData = new double[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test Failure Conditions; data remains unchanged so only examine lengths
      assertEquals(-1, dataBufferEntry.cropData(-1, nPoints));
      assertEquals(nPoints, dataBufferEntry.getData().length);
      assertEquals(-1, dataBufferEntry.cropData(0, nPoints + 1));
      assertEquals(nPoints, dataBufferEntry.getData().length);

      // Test unchanged size
      assertEquals(nPoints, dataBufferEntry.cropData(0, nPoints - 1));
      assertEquals(nPoints, dataBufferEntry.getData().length);

      // Verify data integrity
      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      // Test cropping from end     
      assertEquals(nPoints - 100, dataBufferEntry.cropData(0, nPoints - 101));

      // Verify data integrity
      for (int i = 0; i < nPoints - 100; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      // Restore dataBufferEntry to original state.     
      dataBufferEntry.enlargeBufferSize(nPoints);
      for (int i = 0; i < nPoints; i++)
      {
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test cropping from beginning
      assertEquals(nPoints - 100, dataBufferEntry.cropData(100, nPoints - 1));

      // Verify data integrity
      for (int i = 0; i < dataBufferEntry.getData().length; i++)
      {
         assertEquals(tempData[100 + i], dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testCutData()
   {
      int nPoints = 1000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double[] tempData = new double[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test Failure Conditions; data remains unchanged so only examine lengths
      assertEquals(-1, dataBufferEntry.cropData(-1, nPoints));
      assertEquals(nPoints, dataBufferEntry.getData().length);
      assertEquals(-1, dataBufferEntry.cropData(0, nPoints + 1));
      assertEquals(nPoints, dataBufferEntry.getData().length);

      // Test unchanged size
      assertEquals(-1, dataBufferEntry.cutData(nPoints/2 + 1, nPoints/2 - 1));
      assertEquals(nPoints, dataBufferEntry.getData().length);

      // Verify data integrity
      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      // Test cut one point in the middle:
      int cutPoint = nPoints/2;
      int sizeAfterCut = dataBufferEntry.cutData(cutPoint, cutPoint);
      assertEquals(nPoints-1, sizeAfterCut);
      assertEquals(nPoints-1, dataBufferEntry.getData().length);
      
      // Verify data integrity
      for (int i = 0; i < cutPoint; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }
      
      for (int i = cutPoint; i < nPoints-1; i++)
      {
         assertEquals(tempData[i+1], dataBufferEntry.getData()[i], 0);
      }
      
   // Restore dataBufferEntry to original state.     
      dataBufferEntry.enlargeBufferSize(nPoints);
      for (int i = 0; i < nPoints; i++)
      {
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      // Test cutting at beginning   
      sizeAfterCut = dataBufferEntry.cutData(0, 2);
      assertEquals(nPoints - 3, sizeAfterCut);

      // Verify data integrity
      for (int i = 0; i < nPoints-3; i++)
      {
         assertEquals(tempData[i+3], dataBufferEntry.getData()[i], 0);
      }

      // Restore dataBufferEntry to original state.     
      dataBufferEntry.enlargeBufferSize(nPoints);
      for (int i = 0; i < nPoints; i++)
      {
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test cutting at end
      sizeAfterCut = dataBufferEntry.cutData(nPoints - 3, nPoints-1);
      assertEquals(nPoints - 3, sizeAfterCut);

      // Verify data integrity
      for (int i = 0; i < dataBufferEntry.getData().length; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testPackData()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double[] tempData = new double[nPoints];
      int newStartIndex = random.nextInt(nPoints - 1);
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }

      // Test Bad Start Index, data should be unchanged
      dataBufferEntry.packData(-1);
      for (int i = 0; i < nPoints - 100; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      dataBufferEntry.packData(nPoints + 10);
      for (int i = 0; i < nPoints - 100; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }

      // Test packing
      dataBufferEntry.packData(newStartIndex);

      for (int i = 0; i < (nPoints - 1) - newStartIndex; i++)
      {
         assertEquals(tempData[newStartIndex + i], dataBufferEntry.getData()[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMax()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      int tempInteger = random.nextInt(500) + 11;
      doubleYoVariable.set(tempInteger);
      dataBufferEntry.setDataAtIndexToYoVariableValue(0);
      doubleYoVariable.set(tempInteger + 10);
      dataBufferEntry.setDataAtIndexToYoVariableValue(1);
      doubleYoVariable.set(tempInteger - 10);
      dataBufferEntry.setDataAtIndexToYoVariableValue(2);
      assertEquals(tempInteger + 10, dataBufferEntry.getMax(), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMin()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      int tempInteger = random.nextInt(500) + 11;
      doubleYoVariable.set(tempInteger);
      dataBufferEntry.setDataAtIndexToYoVariableValue(0);
      doubleYoVariable.set(tempInteger + 10);
      dataBufferEntry.setDataAtIndexToYoVariableValue(1);
      doubleYoVariable.set(tempInteger - 10);
      dataBufferEntry.setDataAtIndexToYoVariableValue(2);
      assertEquals(0, dataBufferEntry.getMin(), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMinMaxWithNaN()
   {
      int nPoints = 10000;
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      for (int i = 0; i < 100; i++)
      {
         doubleYoVariable.set(Double.NaN);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);         
      }
      assertEquals(0.0, dataBufferEntry.getMin(), 0.0);
      assertEquals(0.0, dataBufferEntry.getMax(), 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testMinMaxWithNaN2()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      for (int i = 0; i < 100; i++)
      {
         if (i == 50)
            doubleYoVariable.set(Double.NaN);
         else
            doubleYoVariable.set(random.nextDouble());
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      assertFalse(Double.isNaN(dataBufferEntry.getMin()));
      assertFalse(Double.isNaN(dataBufferEntry.getMax()));
      assertTrue(dataBufferEntry.getMin() <= dataBufferEntry.getMax());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testResetMinMaxChanged()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      int tempInteger = random.nextInt(500) + 11;
      doubleYoVariable.set(tempInteger);
      dataBufferEntry.setDataAtIndexToYoVariableValue(0);
      assertTrue(dataBufferEntry.minMaxChanged());
      dataBufferEntry.resetMinMaxChanged();
      assertFalse(dataBufferEntry.minMaxChanged());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testSetData()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double tempDouble = (double) random.nextInt(20000) / (double) random.nextInt(30);
      int randomIndex = random.nextInt(nPoints);
      dataBufferEntry.setData(tempDouble, randomIndex);
      
      assertEquals(tempDouble, dataBufferEntry.getData()[randomIndex], 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetWindowedData()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      double tempData[] = new double[nPoints];
      int randomIndex = random.nextInt(nPoints-1);
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) random.nextInt(20000) / (double) random.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      double tempDataSubset[] = new double[nPoints - randomIndex];
      for(int i = 0; i < tempDataSubset.length; i++)
      {
         tempDataSubset[i] = tempData[randomIndex+i];
      }
      
      double windowedData[] = dataBufferEntry.getWindowedData(randomIndex, /*nPoints-1,*/ nPoints - randomIndex);
      
      for(int i = 0; i < windowedData.length; i++)
      {
         assertEquals(tempDataSubset[i], windowedData[i], 0);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testEnableAutoScale()
   {
      int nPoints = 10000;
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      dataBufferEntry.enableAutoScale(true);
      assertTrue(dataBufferEntry.isAutoScaleEnabled());
      dataBufferEntry.enableAutoScale(false);
      assertFalse(dataBufferEntry.isAutoScaleEnabled());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMaxWithParameters()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      for (int i = 0; i < nPoints; i++)
      {
         doubleYoVariable.set((double) random.nextInt(20000) / (double) random.nextInt(30));
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      double oldMax = dataBufferEntry.getMax();
      double newMax = oldMax + 100;
      
      dataBufferEntry.setData(newMax, 200);
      dataBufferEntry.setData(oldMax, 400);
      
      assertEquals(newMax, dataBufferEntry.getMax(150,250,150,250), 0);
      assertEquals(newMax, dataBufferEntry.getMax(150,0,150,250), 0);
      assertEquals(newMax, dataBufferEntry.getMax(500,250,150,250), 0);
      
      assertEquals(oldMax, dataBufferEntry.getMax(350,450,350,450), 0);
      assertEquals(oldMax, dataBufferEntry.getMax(350,0,350,450), 0);
      assertEquals(oldMax, dataBufferEntry.getMax(500,450,350,450), 0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testGetMinWithParameters()
   {
      int nPoints = 10000;
      Random random = new Random();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      DataBufferEntry dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      
      for (int i = 0; i < nPoints; i++)
      {
         doubleYoVariable.set((double) random.nextInt(20000) / (double) random.nextInt(30));
         dataBufferEntry.setDataAtIndexToYoVariableValue(i);
      }
      
      double oldMin = dataBufferEntry.getMin();
      double newMin = oldMin - 100;
      
      dataBufferEntry.setData(newMin, 200);
      dataBufferEntry.setData(oldMin, 400);
      
      assertEquals(newMin, dataBufferEntry.getMin(150,250,150,250), 0);
      assertEquals(newMin, dataBufferEntry.getMin(150,0,150,250), 0);
      assertEquals(newMin, dataBufferEntry.getMin(500,250,150,250),0);
      
      assertEquals(oldMin, dataBufferEntry.getMin(350,450,350,450), 0);
      assertEquals(oldMin, dataBufferEntry.getMin(350,0,350,450), 0);
      assertEquals(oldMin, dataBufferEntry.getMin(500,450,350,450), 0);
   }
   
}
