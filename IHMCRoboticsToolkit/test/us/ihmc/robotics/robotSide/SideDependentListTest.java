package us.ihmc.robotics.robotSide;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class SideDependentListTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCommonUse()
   {
      double leftFavoriteNumber = Math.PI;
      double rightFavoriteNumber = Math.E;
      
      SideDependentList<Double> sideDependentList = new SideDependentList<Double>();
      sideDependentList.set(RobotSide.LEFT, leftFavoriteNumber);
      sideDependentList.set(RobotSide.RIGHT, rightFavoriteNumber);

      double epsilon = 0.0;
      assertEquals(leftFavoriteNumber, sideDependentList.get(RobotSide.LEFT), epsilon);
      assertEquals(rightFavoriteNumber, sideDependentList.get(RobotSide.RIGHT), epsilon);
      
      double sum = 0.0;
      for (Double doubleValue : sideDependentList)
      {
         sum = sum + doubleValue;
      }
      epsilon = 1e-14;
      assertEquals(leftFavoriteNumber + rightFavoriteNumber, sum, epsilon);
      
      System.out.println(sideDependentList);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testWitGenericObjects()
   {
      ArrayList<Integer> leftObject = new ArrayList<Integer>();
      LinkedList<Boolean> rightObject = new  LinkedList<Boolean>();
      
      @SuppressWarnings({ "rawtypes", "unchecked" })
      SideDependentList sideDependentList = new SideDependentList(leftObject, rightObject);

      assertTrue(leftObject == sideDependentList.get(RobotSide.LEFT));
      assertTrue(rightObject == sideDependentList.get(RobotSide.RIGHT));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000) 
   public void testCopyConstructor()
   {
      Object leftObject = new Object();
      Object rightObject = new Object();
      
      SideDependentList<Object> sideDependentList = new SideDependentList<Object>(leftObject, rightObject);
      SideDependentList<Object> sideDependentListCopy = new SideDependentList<Object>(sideDependentList);
      
      assertTrue(sideDependentList.get(RobotSide.LEFT) == leftObject);
      assertTrue(sideDependentList.get(RobotSide.RIGHT) == rightObject);

      assertTrue(sideDependentList.get(RobotSide.LEFT) == sideDependentListCopy.get(RobotSide.LEFT));
      assertTrue(sideDependentList.get(RobotSide.RIGHT) == sideDependentListCopy.get(RobotSide.RIGHT));
      assertTrue(sideDependentList.get(RobotSide.LEFT) != sideDependentListCopy.get(RobotSide.RIGHT));
      assertTrue(sideDependentList.get(RobotSide.RIGHT) != sideDependentListCopy.get(RobotSide.LEFT));
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = IndexOutOfBoundsException.class)
   public void testIndexOutOfBoundsException()
   {
      Object leftObject = new Object();
      Object rightObject = new Object();
      
      SideDependentList<Object> sideDependentList = new SideDependentList<Object>(leftObject, rightObject);
      
      Iterator<Object> iterator = sideDependentList.iterator();
      
      while(iterator.hasNext())
      {
         iterator.next();
      }
      
      iterator.next();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = UnsupportedOperationException.class)
   public void testIteratorCannotRemove()
   {
      Object leftObject = new Object();
      Object rightObject = new Object();
      
      SideDependentList<Object> sideDependentList = new SideDependentList<Object>(leftObject, rightObject);
      
      Iterator<Object> iterator = sideDependentList.iterator();
      iterator.remove();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSet()
   {
      Object leftObject = new Object();
      Object rightObject = new Object();
      
      SideDependentList<Object> sideDependentList = new SideDependentList<Object>(leftObject, rightObject);
      SideDependentList<Object> sideDependentListCopy = new SideDependentList<Object>();
     
      sideDependentListCopy.set(sideDependentList);
      
      assertTrue(sideDependentList.get(RobotSide.LEFT) == sideDependentListCopy.get(RobotSide.LEFT));
      assertTrue(sideDependentList.get(RobotSide.RIGHT) == sideDependentListCopy.get(RobotSide.RIGHT));
      
      assertTrue(leftObject == sideDependentListCopy.get(RobotSide.LEFT));
      assertTrue(rightObject == sideDependentListCopy.get(RobotSide.RIGHT));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateListOfHashMaps()
   {
      SideDependentList<Map<Double,String>> sideDependentList = SideDependentList.createListOfHashMaps();
      
      LinkedHashMap<Double,String> leftHashMap = new LinkedHashMap<Double, String>();
      LinkedHashMap<Double,String> rightHashMap = new LinkedHashMap<Double, String>();
      
      leftHashMap.put(1.0, "1.0");
      leftHashMap.put(2.0, "2.0");
      rightHashMap.put(3.0, "3.0");
      rightHashMap.put(4.0, "4.0");
      
      sideDependentList.set(RobotSide.LEFT, leftHashMap);
      sideDependentList.set(RobotSide.RIGHT, rightHashMap);
      
      assertEquals(sideDependentList.get(RobotSide.LEFT).get(1.0), "1.0");
      assertEquals(sideDependentList.get(RobotSide.LEFT).get(2.0), "2.0");
      assertEquals(sideDependentList.get(RobotSide.RIGHT).get(3.0), "3.0");
      assertEquals(sideDependentList.get(RobotSide.RIGHT).get(4.0), "4.0");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateListOfEnumMaps()
   {
      SideDependentList<EnumMap<IceCreamFlavor,String>> sideDependentList = SideDependentList.createListOfEnumMaps(IceCreamFlavor.class);
      
      EnumMap<IceCreamFlavor,String> leftEnumMap = new EnumMap<IceCreamFlavor, String>(IceCreamFlavor.class);
      EnumMap<IceCreamFlavor,String> rightEnumMap = new EnumMap<IceCreamFlavor, String>(IceCreamFlavor.class);
      
      leftEnumMap.put(IceCreamFlavor.CHOCOLATE, "YummyChocolate");
      leftEnumMap.put(IceCreamFlavor.VANILLA, "YummyVanilla");
      leftEnumMap.put(IceCreamFlavor.TOOTYFRUITY, "YummyTootyFruity");
     
      rightEnumMap.put(IceCreamFlavor.CHOCOLATE, "ChocolateWow");
      
      sideDependentList.put(RobotSide.LEFT, leftEnumMap);
      sideDependentList.put(RobotSide.RIGHT, rightEnumMap);
      
      assertEquals("YummyChocolate", sideDependentList.get(RobotSide.LEFT).get(IceCreamFlavor.CHOCOLATE));
      assertEquals("YummyVanilla", sideDependentList.get(RobotSide.LEFT).get(IceCreamFlavor.VANILLA));
      assertEquals("YummyTootyFruity", sideDependentList.get(RobotSide.LEFT).get(IceCreamFlavor.TOOTYFRUITY));
      
      assertEquals("ChocolateWow", sideDependentList.get(RobotSide.RIGHT).get(IceCreamFlavor.CHOCOLATE));
      assertEquals(null, sideDependentList.get(RobotSide.RIGHT).get(IceCreamFlavor.VANILLA));
      assertEquals(null, sideDependentList.get(RobotSide.RIGHT).get(IceCreamFlavor.TOOTYFRUITY));
   }
   
   private enum IceCreamFlavor
   {
      CHOCOLATE, VANILLA, TOOTYFRUITY
   }
}
