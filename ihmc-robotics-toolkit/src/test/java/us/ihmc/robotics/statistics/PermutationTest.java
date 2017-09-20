package us.ihmc.robotics.statistics;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class PermutationTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testNumberOfPossiblePermutations()
   {
      // Test: calculates the number of possible permutations for different
      // number of sets and different set sizes

      ArrayList<String> set1 = new ArrayList<String>();
      set1.add("a");
      set1.add("b");

      ArrayList<String> set2 = new ArrayList<String>();
      set2.add("c");
      set2.add("d");

      ArrayList<String> set3 = new ArrayList<String>();
      set3.add("e");
      set3.add("f");

      ArrayList<String> set4 = new ArrayList<String>();
      set4.add("g");
      set4.add("h");
      set4.add("i");

      ArrayList<ArrayList<String>> toBePermutated = new ArrayList<ArrayList<String>>();
      toBePermutated.add(set1);
      toBePermutated.add(set2);

      PermutationRecursive<String> testPermutation = new PermutationRecursive<String>();
      assertEquals(4, testPermutation.numberOfPossiblePermutations(toBePermutated));

      toBePermutated.add(set3);
      assertEquals(8, testPermutation.numberOfPossiblePermutations(toBePermutated));

      toBePermutated.add(set4);
      assertEquals(24, testPermutation.numberOfPossiblePermutations(toBePermutated));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCreateResultContainer()
   {
      // Test: Tests for the size of the Results array created from an input
      // set

      ArrayList<String> set1 = new ArrayList<String>();
      set1.add("a");
      set1.add("b");

      ArrayList<String> set2 = new ArrayList<String>();
      set2.add("c");
      set2.add("d");

      ArrayList<String> set3 = new ArrayList<String>();
      set3.add("e");
      set3.add("f");

      ArrayList<ArrayList<String>> toBePermutated = new ArrayList<ArrayList<String>>();
      toBePermutated.add(set1);
      toBePermutated.add(set2);
      toBePermutated.add(set3);

      PermutationRecursive<String> testPermutation = new PermutationRecursive<String>();
      @SuppressWarnings("unused")
      int permutatedSize = testPermutation.numberOfPossiblePermutations(toBePermutated);
      ArrayList<ArrayList<String>> result = testPermutation.createResultContainer(8);
      assertEquals(8, result.size());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPermutate()
   {
      // Test: Tests for the permutated resulting set to be right. It is also
      // testing for different set sizes and number of sets

      ArrayList<String> set1 = new ArrayList<String>();
      set1.add("a");
      set1.add("b");

      ArrayList<String> set2 = new ArrayList<String>();
      set2.add("c");
      set2.add("d");

      ArrayList<String> set3 = new ArrayList<String>();
      set3.add("e");
      set3.add("f");

      ArrayList<ArrayList<String>> toBePermutated = new ArrayList<ArrayList<String>>();
      toBePermutated.clear();
      toBePermutated.add(set1);
      toBePermutated.add(set2);
      toBePermutated.add(set3);

      ArrayList<ArrayList<String>> result = new ArrayList<ArrayList<String>>();

      PermutationRecursive<String> testPermutation = new PermutationRecursive<String>();
      int permutatedSize = testPermutation.numberOfPossiblePermutations(toBePermutated);
      result = testPermutation.createResultContainer(permutatedSize);

      int position[] = new int[2];
      position[0] = 0;
      position[1] = 0;

      int rowIndicator[] = new int[toBePermutated.size()];

      for (int i = 0; i < toBePermutated.size(); i++)
      {
         rowIndicator[i] = 0;
      }

      // Call Permutate and Assert!
      // System.out.println("To be Permutated: " + toBePermutated);
      result = testPermutation.permutate(position, rowIndicator, 0, toBePermutated, result);

      assertEquals("a", result.get(0).get(0));
      assertEquals("c", result.get(1).get(1));
      assertEquals("e", result.get(2).get(2));

      assertEquals("b", result.get(5).get(0));
      assertEquals("d", result.get(6).get(1));
      assertEquals("f", result.get(7).get(2));

      // Test 2

      // Clear All Containers for a new test
      set1.clear();
      set2.clear();
      set3.clear();
      toBePermutated.clear();
      result.clear();

      set1.add("a");
      set1.add("b");
      set1.add("c");

      set2.add("d");
      set2.add("e");
      set2.add("f");

      set3.add("g");
      set3.add("h");
      set3.add("i");

      toBePermutated.add(set1);
      toBePermutated.add(set2);
      toBePermutated.add(set3);

      permutatedSize = testPermutation.numberOfPossiblePermutations(toBePermutated);
      result = testPermutation.createResultContainer(permutatedSize);

      position[0] = 0;
      position[1] = 0;

      // Clear RowIndicator
      for (int i = 0; i < toBePermutated.size(); i++)
      {
         rowIndicator[i] = 0;
      }

      // Call Permutate and Assert!
      // System.out.println("To be Permutated: " + toBePermutated);
      result = testPermutation.permutate(position, rowIndicator, 0, toBePermutated, result);

      assertEquals("a", result.get(0).get(0));
      assertEquals("d", result.get(1).get(1));
      assertEquals("i", result.get(2).get(2));

      assertEquals("a", result.get(5).get(0));
      assertEquals("f", result.get(6).get(1));
      assertEquals("h", result.get(7).get(2));

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDifferentElementTypes()
   {
      // Test: Tests for the permutation to be accomplished correctly using
      // different element types (any object).

      ArrayList<Object> set1 = new ArrayList<Object>();
      set1.add(1);
      set1.add("a");

      ArrayList<Object> set2 = new ArrayList<Object>();
      set2.add(2);
      set2.add("b");

      ArrayList<ArrayList<Object>> toBePermutated = new ArrayList<ArrayList<Object>>();
      toBePermutated.clear();
      toBePermutated.add(set1);
      toBePermutated.add(set2);

      ArrayList<ArrayList<Object>> result = new ArrayList<ArrayList<Object>>();

      PermutationRecursive<Object> testPermutation = new PermutationRecursive<Object>();
      int permutatedSize = testPermutation.numberOfPossiblePermutations(toBePermutated);
      result = testPermutation.createResultContainer(permutatedSize);

      int position[] = new int[2];
      position[0] = 0;
      position[1] = 0;

      int rowIndicator[] = new int[toBePermutated.size()];

      for (int i = 0; i < toBePermutated.size(); i++)
      {
         rowIndicator[i] = 0;
      }

      // Call Permutate and Assert!
      // System.out.println("To be Permutated: " + toBePermutated);
      result = testPermutation.permutate(position, rowIndicator, 0, toBePermutated, result);

      assertEquals(1, result.get(0).get(0));
      assertEquals("b", result.get(1).get(1));
      assertEquals(2, result.get(2).get(1));

   }

}
