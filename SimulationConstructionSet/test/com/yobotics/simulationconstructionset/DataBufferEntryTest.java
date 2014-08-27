package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class DataBufferEntryTest
{

   private DataBufferEntry dataBufferEntry;
   private DoubleYoVariable doubleYoVariable;
   private int nPoints;
   private Random rng;

   @Before
   public void setUp()
   {
      nPoints = 10000;
      rng = new Random();
      doubleYoVariable = new DoubleYoVariable("doubleYoVariable", null);
      doubleYoVariable.set(0);
      dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
   }

   @After
   public void tearDown()
   {
      doubleYoVariable = null;
      dataBufferEntry = null;
      rng = null;
   }

   @Test
   public void testGetVal()
   {
      double tempDouble = (double) rng.nextInt(20000) / (double) rng.nextInt(30);
      doubleYoVariable.set(tempDouble);
      assertEquals(tempDouble, dataBufferEntry.getVariableValueAsADouble(), 0);
   }

   @Test
   public void testTickAndUpdate()
   {
      double epsilon = 0;
      double[] tempData = new double[nPoints];
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) rng.nextInt(20000) / (double) rng.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.tickAndUpdate(i);
      }

      double[] data = dataBufferEntry.getData();

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], data[i], epsilon);
      }
   }

   @Test
   public void testUpdateValue()
   {
      double epsilon = 0;
      double[] tempData = new double[nPoints];
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) rng.nextInt(20000) / (double) rng.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.tickAndUpdate(i);
      }

      for (int i = 0; i < nPoints; i++)
      {
         dataBufferEntry.updateValue(i);
         assertEquals(doubleYoVariable.getValueAsDouble(), dataBufferEntry.getData()[i], epsilon);
      }
   }

   @Test
   public void testCheckIfDataIsEqual()
   {
      double temp;
      double epsilon = 0;
      DoubleYoVariable doubleYoVariable2 = new DoubleYoVariable("doubleYoVariable2", null);
      doubleYoVariable2.set(0);
      DataBufferEntry entry2 = new DataBufferEntry(doubleYoVariable2, nPoints);

      for (int i = 0; i < nPoints; i++)
      {
         temp = (double) rng.nextInt(20000) / (double) rng.nextInt(30);

         doubleYoVariable.set(temp);
         doubleYoVariable2.set(temp);

         dataBufferEntry.tickAndUpdate(i);
         entry2.tickAndUpdate(i);
      }

      assertTrue(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints - 1, epsilon));

      dataBufferEntry = entry2 = null;

      dataBufferEntry = new DataBufferEntry(doubleYoVariable, nPoints);
      entry2 = new DataBufferEntry(doubleYoVariable2, nPoints);

      for (int i = 0; i < nPoints; i++)
      {
         temp = (double) rng.nextInt(20000) / (double) rng.nextInt(30);

         doubleYoVariable.set(temp);
         doubleYoVariable2.set(-temp);

         dataBufferEntry.tickAndUpdate(i);
         entry2.tickAndUpdate(i);
      }

      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints - 1, epsilon));
      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, 0, nPoints, epsilon)); // outPoint out of bounds
      assertFalse(dataBufferEntry.checkIfDataIsEqual(entry2, nPoints, nPoints - 1, epsilon)); //inPoint out of bounds
   }

   @Test
   public void testGetMinAndMaxScaling()
   {
      double minScaling = rng.nextDouble();
      double maxScaling = rng.nextDouble() + 2;
      dataBufferEntry.setManualScaling(minScaling, maxScaling);

      assertEquals(minScaling, dataBufferEntry.getManualMinScaling(), 0);
      assertEquals(maxScaling, dataBufferEntry.getManualMaxScaling(), 0);
   }

   @Test
   public void testGetVariable()
   {
      assertEquals(doubleYoVariable, dataBufferEntry.getVariable());
   }

   @Test
   public void testCopyValueThrough()
   {
      double tempDouble = rng.nextDouble();
      doubleYoVariable.set(tempDouble);
      dataBufferEntry.tickAndUpdate(0);
      dataBufferEntry.copyValueThrough();

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempDouble, dataBufferEntry.getData()[i], 0);
      }
   }

   @Test
   public void testEnlargeBufferSize()
   {
      double[] tempData = new double[nPoints];
      int newSizeDelta = rng.nextInt(100);

      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) rng.nextInt(20000) / (double) rng.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.tickAndUpdate(i);
      }
      dataBufferEntry.enlargeBufferSize(nPoints + newSizeDelta);

      assertEquals(nPoints + newSizeDelta, dataBufferEntry.getData().length);

      for (int i = 0; i < nPoints; i++)
      {
         assertEquals(tempData[i], dataBufferEntry.getData()[i], 0);
      }
   }

   @Test
   public void testCropData()
   {
      double[] tempData = new double[nPoints];

      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) rng.nextInt(20000) / (double) rng.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.tickAndUpdate(i);
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
         dataBufferEntry.tickAndUpdate(i);
      }

      // Test cropping from beginning
      assertEquals(nPoints - 100, dataBufferEntry.cropData(100, nPoints - 1));

      // Verify data integrity
      for (int i = 0; i < dataBufferEntry.getData().length; i++)
      {
         assertEquals(tempData[100 + i], dataBufferEntry.getData()[i], 0);
      }
   }

   @Test
   public void testPackData()
   {
      double[] tempData = new double[nPoints];
      int newStartIndex = rng.nextInt(nPoints - 1);
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) rng.nextInt(20000) / (double) rng.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.tickAndUpdate(i);
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

   @Test
   public void testGetMax()
   {
      int tempInteger = rng.nextInt(500) + 11;
      doubleYoVariable.set(tempInteger);
      dataBufferEntry.tickAndUpdate(0);
      doubleYoVariable.set(tempInteger + 10);
      dataBufferEntry.tickAndUpdate(1);
      doubleYoVariable.set(tempInteger - 10);
      dataBufferEntry.tickAndUpdate(2);
      assertEquals(tempInteger + 10, dataBufferEntry.getMax(), 0);
   }

   @Test
   public void testGetMin()
   {
      int tempInteger = rng.nextInt(500) + 11;
      doubleYoVariable.set(tempInteger);
      dataBufferEntry.tickAndUpdate(0);
      doubleYoVariable.set(tempInteger + 10);
      dataBufferEntry.tickAndUpdate(1);
      doubleYoVariable.set(tempInteger - 10);
      dataBufferEntry.tickAndUpdate(2);
      assertEquals(0, dataBufferEntry.getMin(), 0);
   }

   @Test
   public void testMinMaxWithNaN()
   {
      for (int i = 0; i < 100; i++)
      {
         doubleYoVariable.set(Double.NaN);
         dataBufferEntry.tickAndUpdate(i);         
      }
      assertEquals(0.0, dataBufferEntry.getMin(), 0.0);
      assertEquals(0.0, dataBufferEntry.getMax(), 0.0);
   }

   @Test
   public void testMinMaxWithNaN2()
   {
      for (int i = 0; i < 100; i++)
      {
         if (i == 50)
            doubleYoVariable.set(Double.NaN);
         else
            doubleYoVariable.set(rng.nextDouble());
         dataBufferEntry.tickAndUpdate(i);
      }
      assertFalse(Double.isNaN(dataBufferEntry.getMin()));
      assertFalse(Double.isNaN(dataBufferEntry.getMax()));
      assertTrue(dataBufferEntry.getMin() <= dataBufferEntry.getMax());
   }
   
   @Test
   public void testResetMinMaxChanged()
   {
      int tempInteger = rng.nextInt(500) + 11;
      doubleYoVariable.set(tempInteger);
      dataBufferEntry.tickAndUpdate(0);
      assertTrue(dataBufferEntry.minMaxChanged());
      dataBufferEntry.resetMinMaxChanged();
      assertFalse(dataBufferEntry.minMaxChanged());
   }
   
   @Test
   public void testSetData()
   {
      double tempDouble = (double) rng.nextInt(20000) / (double) rng.nextInt(30);
      int randomIndex = rng.nextInt(nPoints);
      dataBufferEntry.setData(tempDouble, randomIndex);
      
      assertEquals(tempDouble, dataBufferEntry.getData()[randomIndex], 0);
   }
   
   @Test
   public void testGetWindowedData()
   {
      double tempData[] = new double[nPoints];
      int randomIndex = rng.nextInt(nPoints-1);
      for (int i = 0; i < nPoints; i++)
      {
         tempData[i] = (double) rng.nextInt(20000) / (double) rng.nextInt(30);
         doubleYoVariable.set(tempData[i]);
         dataBufferEntry.tickAndUpdate(i);
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
   
   @Test
   public void testEnableAutoScale()
   {
      dataBufferEntry.enableAutoScale(true);
      assertTrue(dataBufferEntry.isAutoScaleEnabled());
      dataBufferEntry.enableAutoScale(false);
      assertFalse(dataBufferEntry.isAutoScaleEnabled());
   }
   
   @Test
   public void testGetMaxWithParameters()
   {
      for (int i = 0; i < nPoints; i++)
      {
         doubleYoVariable.set((double) rng.nextInt(20000) / (double) rng.nextInt(30));
         dataBufferEntry.tickAndUpdate(i);
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
   
   @Test
   public void testGetMinWithParameters()
   {
      for (int i = 0; i < nPoints; i++)
      {
         doubleYoVariable.set((double) rng.nextInt(20000) / (double) rng.nextInt(30));
         dataBufferEntry.tickAndUpdate(i);
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
