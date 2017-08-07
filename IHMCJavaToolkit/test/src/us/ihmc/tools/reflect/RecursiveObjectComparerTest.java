package us.ihmc.tools.reflect;

import static org.junit.Assert.assertEquals;

import java.lang.reflect.Field;
import java.util.AbstractList;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;

public class RecursiveObjectComparerTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testStringEqual() throws IllegalArgumentException, IllegalAccessException, SecurityException, NoSuchFieldException
   {
      String string1 = "abcd";
      String string2 = new String(string1);

      Field hashField = String.class.getDeclaredField("hash");
      ArrayList<Field> fieldsToIgnore = new ArrayList<Field>();
      fieldsToIgnore.add(hashField);
      compareAndAssert(string1, string2, true, fieldsToIgnore);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testStringNotEqual() throws IllegalArgumentException, IllegalAccessException
   {
      String string1 = "abcd";
      String string2 = "efgh";

      compareAndAssert(string1, string2, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDifferentTypesNotEqual() throws IllegalArgumentException, IllegalAccessException
   {
      boolean b = true;
      char c = 'a';
      double d = 1.5;
      String s = "hello";
      ArrayList<Double> listOfDoubles = new ArrayList<Double>();
      ArrayList<Integer> listOfIntegers = new ArrayList<Integer>();

      compareAndAssert(b, c, false);
      compareAndAssert(b, d, false);
      compareAndAssert(b, s, false);
      compareAndAssert(b, listOfDoubles, false);
      compareAndAssert(b, listOfIntegers, false);

      compareAndAssert(c, b, false);
      compareAndAssert(c, d, false);
      compareAndAssert(c, s, false);
      compareAndAssert(c, listOfDoubles, false);
      compareAndAssert(c, listOfIntegers, false);

      compareAndAssert(d, b, false);
      compareAndAssert(d, c, false);
      compareAndAssert(d, s, false);
      compareAndAssert(d, listOfDoubles, false);
      compareAndAssert(d, listOfIntegers, false);

      compareAndAssert(s, b, false);
      compareAndAssert(s, c, false);
      compareAndAssert(s, d, false);
      compareAndAssert(s, listOfDoubles, false);
      compareAndAssert(s, listOfIntegers, false);

      compareAndAssert(listOfDoubles, b, false);
      compareAndAssert(listOfDoubles, c, false);
      compareAndAssert(listOfDoubles, d, false);
      compareAndAssert(listOfDoubles, s, false);
      // Empty ArrayLists of different types are the same! compareAndAssert(listOfDoubles, listOfIntegers, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDoubleArraysDifferentInObject() throws IllegalArgumentException, IllegalAccessException
   {
      Object objectOne = new ObjectWithADoubleArray(new double[] {1.0, 2.0, 3.0});
      Object objectTwo = new ObjectWithADoubleArray(new double[] {4.0, 5.0, 6.0});
      Object objectThree = new ObjectWithADoubleArray(new double[] {1.0, 2.0, 3.0});

      //      System.out.println("testDoubleArraysDifferentInObject: " + compareAndAssert(objectOne, objectTwo, false));
      compareAndAssert(objectTwo, objectOne, false);
      compareAndAssert(objectOne, objectThree, true);
      compareAndAssert(objectThree, objectOne, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBuriedDoubleArraysDifferentInObject() throws IllegalArgumentException, IllegalAccessException
   {
      Object objectOne = new ObjectWithADoubleArrayBuried(new double[] {1.0, 2.0, 3.0});
      Object objectTwo = new ObjectWithADoubleArrayBuried(new double[] {4.0, 5.0, 6.0});
      Object objectThree = new ObjectWithADoubleArrayBuried(new double[] {1.0, 2.0, 3.0});

      //      System.out.println("testBuriedDoubleArraysDifferentInObject: " + compareAndAssert(objectOne, objectTwo, false));
      compareAndAssert(objectTwo, objectOne, false);
      compareAndAssert(objectOne, objectThree, true);
      compareAndAssert(objectThree, objectOne, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMapEqual() throws IllegalArgumentException, IllegalAccessException
   {
      HashMap<String, Double> map1 = new HashMap<String, Double>();
      map1.put("abcd", 12.0);
      map1.put("efgh", 23.5);

      HashMap<String, Double> map2 = new HashMap<String, Double>(map1);

      compareAndAssert(map1, map2, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMapEqualDifferentOrder() throws IllegalArgumentException, IllegalAccessException
   {
      HashMap<String, Double> map1 = new HashMap<String, Double>();
      map1.put("abcd", 12.0);
      map1.put("efgh", 23.5);

      HashMap<String, Double> map2 = new HashMap<String, Double>();
      map2.put("efgh", 23.5);
      map2.put("abcd", 12.0);

      compareAndAssert(map1, map2, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMapKeysNotEqual() throws IllegalArgumentException, IllegalAccessException
   {
      HashMap<String, Double> map1 = new HashMap<String, Double>();
      map1.put("abcd", 12.0);
      map1.put("efgh", 23.5);

      HashMap<String, Double> map2 = new HashMap<String, Double>();
      map2.put("BLA", 12.0);
      map2.put("efgh", 23.5);

      compareAndAssert(map1, map2, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMapValuesNotEqual() throws IllegalArgumentException, IllegalAccessException
   {
      HashMap<String, Double> map1 = new HashMap<String, Double>();
      map1.put("abcd", 12.0);
      map1.put("efgh", 23.5);

      HashMap<String, Double> map2 = new HashMap<String, Double>();
      map2.put("BLA", 12.0);
      map2.put("efgh", 214434231234.5);

      compareAndAssert(map1, map2, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClearEqualInitially() throws IllegalArgumentException, IllegalAccessException
   {
      double[] array1 = new double[1];
      array1[0] = 1.0;
      double[] array2 = new double[1];
      array2[0] = array1[0];

      int maxDepth = Integer.MAX_VALUE;
      int maxSize = Integer.MAX_VALUE;
      RecursiveObjectComparer comparer = new RecursiveObjectComparer(maxDepth, maxSize);
      assertEquals(true, comparer.compare(array1, array2));

      comparer.clear();
      array2[0] = 2.0;
      assertEquals(false, comparer.compare(array1, array2));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClearNotEqualInitially() throws IllegalArgumentException, IllegalAccessException
   {
      double[] array1 = new double[1];
      array1[0] = 1.0;
      double[] array2 = new double[1];
      array2[0] = 2.0;

      int maxDepth = Integer.MAX_VALUE;
      int maxSize = Integer.MAX_VALUE;
      RecursiveObjectComparer comparer = new RecursiveObjectComparer(maxDepth, maxSize);
      assertEquals(false, comparer.compare(array1, array2));

      comparer.clear();
      array2[0] = array1[0];
      assertEquals(true, comparer.compare(array1, array2));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLoopsEqual() throws IllegalArgumentException, IllegalAccessException
   {
      LoopCloser loopCloser1 = createLoop();
      LoopCloser loopCloser2 = createLoop();

      compareAndAssert(loopCloser1, loopCloser2, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLoopsNotEqual() throws IllegalArgumentException, IllegalAccessException
   {
      LoopCloser loopCloser1 = createLoop();
      LoopCloser loopCloser2 = createLoop();
      loopCloser2.getLoopClosers().add(null);

      compareAndAssert(loopCloser1, loopCloser2, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLoopsNotEqual2() throws IllegalArgumentException, IllegalAccessException
   {
      LoopCloser loopCloser1 = createLoop();
      LoopCloser loopCloser2 = createLoop();
      loopCloser2.getLoopClosers().add(loopCloser2);

      compareAndAssert(loopCloser1, loopCloser2, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLoopsNotEqual3() throws IllegalArgumentException, IllegalAccessException
   {
      LoopCloser loopCloser1 = createLoop();
      LoopCloser loopCloser2 = createLoop();
      loopCloser2.getLoopClosers().add(loopCloser1);

      compareAndAssert(loopCloser1, loopCloser2, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testArrayOfPrimitives() throws IllegalArgumentException, IllegalAccessException
   {
      double[] array1 = {2.0, 3.0};
      double[] array2 = {2.0, 3.0};
      double[] array3 = {4.0, 5.0};

      compareAndAssert(array1, array2, true);
      compareAndAssert(array1, array3, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testArrayOfStrings() throws IllegalArgumentException, IllegalAccessException
   {
      String string1 = "abcd";
      String string2 = "efgh";
      String string3 = "ijkl";

      String[] array1 = {string1, string2};
      String[] array2 = {string1, string2};
      String[] array3 = {string1, string3};

      compareAndAssert(array1, array2, true);
      compareAndAssert(array1, array3, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSomePrimitives() throws IllegalArgumentException, IllegalAccessException, SecurityException, NoSuchFieldException
   {
      SomePrimitives somePrimitives1 = new SomePrimitives();
      SomePrimitives somePrimitives2 = somePrimitives1.copy();

      Field hashField = String.class.getDeclaredField("hash");
      ArrayList<Field> fieldsToIgnore = new ArrayList<Field>();
      fieldsToIgnore.add(hashField);

      compareAndAssert(somePrimitives1, somePrimitives2, true, fieldsToIgnore);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTree() throws IllegalArgumentException, IllegalAccessException, SecurityException, NoSuchFieldException
   {
      Tree tree1 = new Tree(1, 1);
      Tree tree2 = tree1.inneficientCopy();

      ArrayList<Field> fieldsToIgnore = new ArrayList<Field>();

      fieldsToIgnore.add(AbstractList.class.getDeclaredField("modCount"));
      compareAndAssert(tree1, tree2, true, fieldsToIgnore);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNaN() throws IllegalArgumentException, IllegalAccessException
   {
      Double d1 = Double.NaN;
      Double d2 = Double.NaN;
      compareAndAssert(d1, d2, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNaNVector() throws IllegalArgumentException, IllegalAccessException
   {
      Vector3D object1 = new Vector3D(Double.NaN, Double.NaN, Double.NaN);
      Vector3D object2 = new Vector3D(Double.NaN, Double.NaN, Double.NaN);
      compareAndAssert(object1, object2, true);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOneNull() throws IllegalArgumentException, IllegalAccessException
   {
      Object object1 = null;
      Object object2 = null;
      compareAndAssert(object1, object2, true);

      object2 = new Object();
      compareAndAssert(object1, object2, false);

      object1 = new Object();
      object2 = null;
      String compareString = compareAndAssert(object1, object2, false);
      System.err.println("testOneNull: " + compareString);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOneNullTwo() throws IllegalArgumentException, IllegalAccessException
   {
      ImmutablePair<Object, Object> pair1 = new ImmutablePair<Object, Object>(null, new Object());
      ImmutablePair<Object, Object> pair2 = new ImmutablePair<Object, Object>(null, new Object());
      compareAndAssert(pair1, pair2, true);

      pair1 = new ImmutablePair<Object, Object>(new Object(), new Object());
      pair2 = new ImmutablePair<Object, Object>(null, new Object());
      compareAndAssert(pair1, pair2, false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIgnoreSomeFields() throws IllegalArgumentException, IllegalAccessException, SecurityException, NoSuchFieldException
   {
      Vector3D firstObject = new Vector3D(0.1, 0.2, 0.3);
      Vector3D secondObject = new Vector3D(0.1, 0.5, 0.3);

      //      StringAndRegularExpressionMatcher fieldsToIgnore = new StringAndRegularExpressionMatcher();
      //      fieldsToIgnore.addExactStringToMatch("public double javax.vecmath.Tuple3d.y");
      //      
      Field yField = Vector3D.class.getDeclaredField("y");
      ArrayList<Field> fieldsToIgnore = new ArrayList<Field>();
      fieldsToIgnore.add(yField);

      compareAndAssert(firstObject, secondObject, false);
      compareAndAssert(firstObject, secondObject, true, fieldsToIgnore);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIgnoreSomeNamedFields() throws IllegalArgumentException, IllegalAccessException, SecurityException, NoSuchFieldException
   {
      ObjectWithAName objectOne = new ObjectWithAName("MyName", 1.1);
      ObjectWithAName objectTwo = new ObjectWithAName("MyName", 1.1);

      compareAndAssert(objectOne, objectTwo, true);
      objectTwo = new ObjectWithAName("MyName", 1.9);
      compareAndAssert(objectOne, objectTwo, false);

      Field valueField = ObjectWithAName.class.getDeclaredField("value");
      ArrayList<Field> fieldsToIgnore = new ArrayList<Field>();
      fieldsToIgnore.add(valueField);

      compareAndAssert(objectOne, objectTwo, true, fieldsToIgnore);

      Field nameField = ObjectWithAName.class.getDeclaredField("name");

      StringFieldMatcher matcher = new StringFieldMatcher();
      matcher.addStringFieldToMatchExactly(ObjectWithAName.class, nameField, "MyName");

      compareAndAssert(objectOne, objectTwo, true, matcher);
   }

   private class ObjectWithAName
   {
      private final String name;
      private final double value;

      public ObjectWithAName(String name, double value)
      {
         this.name = name;
         this.value = value;
      }

      @Override
      public String toString()
      {
         return "name = " + name + ", value = " + value;
      }
   }

   private LoopCloser createLoop()
   {
      ArrayList<LoopCloser> loopClosers = new ArrayList<LoopCloser>();
      LoopCloser loopCloser = new LoopCloser(loopClosers);
      loopClosers.add(loopCloser);

      return loopCloser;
   }

   private String compareAndAssert(Object object1, Object object2, boolean expected) throws IllegalArgumentException, IllegalAccessException
   {
      return compareAndAssert(object1, object2, expected, new ArrayList<Field>(), null);
   }

   private String compareAndAssert(Object objectOne, Object objectTwo, boolean expected, StringFieldMatcher matcherToIgnore)
         throws IllegalArgumentException, IllegalAccessException
   {
      return compareAndAssert(objectOne, objectTwo, expected, new ArrayList<Field>(), matcherToIgnore);
   }

   private String compareAndAssert(Object objectOne, Object objectTwo, boolean expected, ArrayList<Field> fieldsToIgnore)
         throws IllegalArgumentException, IllegalAccessException
   {
      return compareAndAssert(objectOne, objectTwo, expected, fieldsToIgnore, null);
   }

   private String compareAndAssert(Object object1, Object object2, boolean expected, ArrayList<Field> fieldsToIgnore,
                                   StringFieldMatcher stringFieldMatcherToIgnore)
         throws IllegalArgumentException, IllegalAccessException
   {
      int maxDepth = Integer.MAX_VALUE;
      int maxSize = Integer.MAX_VALUE;

      RecursiveObjectComparer comparer = new RecursiveObjectComparer(maxDepth, maxSize);

      // Fields we are ok with being different:
      if (fieldsToIgnore != null)
      {
         comparer.addFieldsToIgnore(fieldsToIgnore);
      }
      if (stringFieldMatcherToIgnore != null)
      {
         comparer.addStringFieldsToIgnore(stringFieldMatcherToIgnore);
      }

      boolean result = comparer.compare(object1, object2);
      if (expected != result)
      {
         System.out.println("Differences:");
         System.out.println(comparer.toString());
      }
      assertEquals(expected, result);

      return comparer.toString();
   }

   private static class LoopCloser
   {
      private ArrayList<LoopCloser> loopClosers;

      public ArrayList<LoopCloser> getLoopClosers()
      {
         return loopClosers;
      }

      public LoopCloser(ArrayList<LoopCloser> loopClosers)
      {
         this.loopClosers = loopClosers;
      }
   }

   private static class SomePrimitives
   {
      private double d;
      private int i;
      private boolean b;
      private String s;

      public SomePrimitives()
      {
         Random random = new Random(1776L);
         d = random.nextDouble();
         i = random.nextInt(1000);
         b = random.nextBoolean();
         s = "" + random.nextDouble();
      }

      public SomePrimitives copy()
      {
         SomePrimitives ret = new SomePrimitives();
         ret.d = this.d;
         ret.i = this.i;
         ret.b = this.b;
         ret.s = new String(this.s);

         return ret;
      }
   }

   private static class Tree
   {
      private ArrayList<Tree> children = new ArrayList<Tree>();
      private double value;
      private final int breadth, depth;

      public Tree(int breadth, int depth)
      {
         value = Math.random();
         this.breadth = breadth;
         this.depth = depth;

         if (depth == 0)
            return;
         for (int i = 0; i < breadth; i++)
         {
            children.add(new Tree(breadth, depth - 1));
         }
      }

      public Tree inneficientCopy()
      {
         Tree ret = new Tree(this.breadth, this.depth);
         ret.value = this.value;

         ret.children.clear();

         if (depth > 0)
         {
            for (int i = 0; i < breadth; i++)
            {
               Tree childToCopy = this.children.get(i);
               Tree copy = childToCopy.inneficientCopy();
               ret.children.add(copy);
            }
         }

         return ret;
      }
   }

   private class ObjectWithADoubleArrayBuried
   {
      @SuppressWarnings("unused")
      private ObjectWithADoubleArray grave;

      public ObjectWithADoubleArrayBuried(double[] xToSet)
      {
         grave = new ObjectWithADoubleArray(xToSet);
      }
   }

   private class ObjectWithADoubleArray
   {
      @SuppressWarnings("unused")
      private double[] x;

      public ObjectWithADoubleArray(double[] xToSet)
      {
         x = Arrays.copyOf(xToSet, xToSet.length);
      }
   }
}
