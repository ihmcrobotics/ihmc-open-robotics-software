package us.ihmc.robotics.statistics;

import java.util.ArrayList;

public class PermutationRecursive<E>
{
   boolean debug = false;

   // Permutation Algorithm
   public ArrayList<ArrayList<E>> permutate(int position[], int rowIndicator[], int resultIndex, ArrayList<ArrayList<E>> toBePermutated, ArrayList<ArrayList<E>> result)
   {
//      int flag = 0;

      if (resultIndex < result.size())
      {

         if (debug)
         {
            System.out.println("Result :" + result);
         }

         position[1] = rowIndicator[position[0]];

         result.get(resultIndex).add(toBePermutated.get(position[0]).get(position[1]));

         // If end of setMain is not reached
         if (position[0] < (toBePermutated.size() - 1))
         {
            // If end of individualSet is not reached
            if (position[1] < toBePermutated.get(position[0]).size())
            {
               position[0]++; // add set
            }
            else // If end of individual set is reached
            {

            }
         }
         else // IF end of setMain is reached
         {
            // If end of individualSet is not reached
            if (position[1] < (toBePermutated.get(position[0]).size() - 1))
            {
               rowIndicator[position[0]]++;
               position[0] = 0;
            }
            else // If end of individual set is reached
            {
               rowIndicator[position[0]] = 0;
               rowIndicator[position[0] - 1]++;
               position[0] = 0;

            }
            resultIndex++;
         }

         // Adjust RowIndicator Indices!
         for (int i = 0; i < rowIndicator.length - 1; i++)
         {

            if (rowIndicator[i] > toBePermutated.get(position[0]).size() - 1)
            {
               rowIndicator[i] = 0;
               rowIndicator[position[0]]++;

            }

         }

         // Recurse
         result = permutate(position, rowIndicator, resultIndex, toBePermutated, result);

      }
      else // Done!!
      {
         if (debug)
         {
            System.out.println("Result :" + result);
         }
      }

      return result;

   }

   public ArrayList<ArrayList<E>> createResultContainer(int permutatedSize)
   {
      ArrayList<ArrayList<E>> result = new ArrayList<ArrayList<E>>();

      for (int i = 0; i < permutatedSize; i++)
      {
         result.add(new ArrayList<E>());
      }
      return result;
   }

   public int numberOfPossiblePermutations(ArrayList<ArrayList<E>> toBePermutated)
   {
      int possiblePermutations = 1;
      for (ArrayList<E> list : toBePermutated)
      {
         possiblePermutations = possiblePermutations * list.size();
      }
      return possiblePermutations;
   }

   public static void main(String[] args)
   {
      new PermutationRecursive<String>();
   }

}
