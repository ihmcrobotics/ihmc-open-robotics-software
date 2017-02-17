package us.ihmc.tools.containers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;
import java.util.Vector;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ContainerToolsTest
{

   private enum TestEnum
   {
      ONE, TWO, THREE;
   }

   private enum AnotherTestEnum
   {
      ONE, TWO, THREE;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateEnumMap()
   {
      EnumMap<TestEnum, Integer> enumMap = ContainerTools.createEnumMap(TestEnum.class);
      assertNotNull(enumMap);

      //      enumMap.put(TestEnum.ONE, 1);
      //      enumMap.put(TestEnum.TWO, 2);
      //      enumMap.put(TestEnum.THREE, 3);
      //      
      //      for(TestEnum e : TestEnum.values())
      //         assertTrue(enumMap.containsKey(e));
      //      
      //      for(int i = 1; i <= 3; i++)
      //         assertTrue(enumMap.containsValue(i));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFindLargestElements()
   {
      ArrayList<Integer> list = new ArrayList<Integer>(Arrays.asList(generateIntegerArray()));

      int maxValueInList = Collections.max(list);

      int largestValue = maxValueInList + 3;
      int secondLargestValue = maxValueInList + 2;
      int thirdLargestValue = maxValueInList + 1;

      list.add(largestValue);
      list.add(secondLargestValue);
      list.add(thirdLargestValue);

      Comparator<Integer> comparator = new Comparator<Integer>()
      {
         @Override
         public int compare(Integer o1, Integer o2)
         {
            if (o1.intValue() > o2.intValue())
               return 1;
            if (o1.intValue() < o2.intValue())
               return -1;
            return 0;
         }
      };

      Set<Integer> setOfThreeLargestInts = ContainerTools.findLargestElements(list, comparator, 3);

      assertTrue(setOfThreeLargestInts.contains(largestValue));
      assertTrue(setOfThreeLargestInts.contains(secondLargestValue));
      assertTrue(setOfThreeLargestInts.contains(thirdLargestValue));
      assertEquals(3, setOfThreeLargestInts.size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testFlatten()
   {
      EnumMap<TestEnum, Integer> map1 = ContainerTools.createEnumMap(TestEnum.class);
      EnumMap<TestEnum, Integer> map2 = ContainerTools.createEnumMap(TestEnum.class);
      EnumMap<TestEnum, Integer> map3 = ContainerTools.createEnumMap(TestEnum.class);
      EnumMap<AnotherTestEnum, EnumMap<TestEnum, Integer>> mapOfMaps = ContainerTools.createEnumMap(AnotherTestEnum.class);

      for (TestEnum e : TestEnum.values())
         map1.put(e, e.ordinal() + 1);

      map2.put(TestEnum.ONE, 3);
      map2.put(TestEnum.TWO, 2);
      map2.put(TestEnum.THREE, 1);

      map3.put(TestEnum.ONE, 2);
      map3.put(TestEnum.TWO, 3);
      map3.put(TestEnum.THREE, 1);

      mapOfMaps.put(AnotherTestEnum.ONE, map1);
      mapOfMaps.put(AnotherTestEnum.TWO, map2);
      mapOfMaps.put(AnotherTestEnum.THREE, map3);

      ArrayList<Integer> listOfFlattenedValues = ContainerTools.flatten(mapOfMaps);

      assertEquals(9, listOfFlattenedValues.size());

      int numberOfThrees, numberOfTwos, numberOfOnes;

      numberOfOnes = numberOfTwos = numberOfThrees = 0;

      for (Integer i : listOfFlattenedValues)
      {
         if (i == 1)
            numberOfOnes++;
         if (i == 2)
            numberOfTwos++;
         if (i == 3)
            numberOfThrees++;
      }

      assertEquals(3, numberOfOnes);
      assertEquals(3, numberOfTwos);
      assertEquals(3, numberOfThrees);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testToArrayOfArrays()
   {
      EnumMap<TestEnum, Double> map1 = ContainerTools.createEnumMap(TestEnum.class);
      EnumMap<TestEnum, Double> map2 = ContainerTools.createEnumMap(TestEnum.class);
      EnumMap<TestEnum, Double> map3 = ContainerTools.createEnumMap(TestEnum.class);
      EnumMap<AnotherTestEnum, EnumMap<TestEnum, Double>> mapOfMaps = ContainerTools.createEnumMap(AnotherTestEnum.class);

      for (TestEnum e : TestEnum.values())
         map1.put(e, (double) e.ordinal() + 1);

      map2.put(TestEnum.ONE, (double) 3);
      map2.put(TestEnum.TWO, (double) 2);
      map2.put(TestEnum.THREE, (double) 1);

      map3.put(TestEnum.ONE, (double) 2);
      map3.put(TestEnum.TWO, (double) 3);
      map3.put(TestEnum.THREE, (double) 1);

      mapOfMaps.put(AnotherTestEnum.ONE, map1);
      mapOfMaps.put(AnotherTestEnum.TWO, map2);
      mapOfMaps.put(AnotherTestEnum.THREE, map3);

      double[][] arrayOfArrays = ContainerTools.toArrayOfArrays(mapOfMaps);

      assertEquals(3, arrayOfArrays.length);
      assertEquals(3, arrayOfArrays[0].length);
      assertEquals(3, arrayOfArrays[1].length);
      assertEquals(3, arrayOfArrays[2].length);

      int numberOfThrees, numberOfTwos, numberOfOnes;

      numberOfOnes = numberOfTwos = numberOfThrees = 0;

      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            if (arrayOfArrays[i][j] < 2)
               numberOfOnes++;
            else if (arrayOfArrays[i][j] > 2)
               numberOfThrees++;
            else
               numberOfTwos++;
         }
      }

      assertEquals(3, numberOfOnes);
      assertEquals(3, numberOfTwos);
      assertEquals(3, numberOfThrees);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAsSortedList()
   {
      Integer[] baseArray = generateIntegerArray();
      Collection<Integer> collection = new Vector<Integer>(Arrays.asList(baseArray));
      
      Arrays.sort(baseArray);
      ArrayList<Integer> sortedArrayList = new ArrayList<Integer>(Arrays.asList(baseArray));
            
      List<Integer> sortedListUsingContainerTools = ContainerTools.asSortedList(collection);
      
      for(Integer i : sortedArrayList)
      {
         assertTrue(sortedListUsingContainerTools.contains(i));
         assertEquals(sortedArrayList.indexOf(i), sortedListUsingContainerTools.indexOf(i));
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRemoveByReference()
   {
      Set<Integer> set = new TreeSet<Integer>(Arrays.asList(generateIntegerArray()));      
      List<Integer> setAsList = new ArrayList<Integer>(set);
      
      Integer intToRemove = setAsList.get(0);
      
      ContainerTools.removeByReference(setAsList, intToRemove);
      
      assertFalse(setAsList.contains(intToRemove));      
   }

   private Integer[] generateIntegerArray()
   {
      Random random = new Random();
      int arraySize = random.nextInt(500) + 1;
      Integer[] array = new Integer[arraySize];

      for (int i = 0; i < arraySize; i++)
         array[i] = random.nextInt();

      return array;
   }
}
