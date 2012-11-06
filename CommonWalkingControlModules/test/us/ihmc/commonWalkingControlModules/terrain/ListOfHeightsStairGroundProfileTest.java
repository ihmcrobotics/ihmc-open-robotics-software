package us.ihmc.commonWalkingControlModules.terrain;

import static org.junit.Assert.*;

import org.junit.Test;

public class ListOfHeightsStairGroundProfileTest
{

   @Test
   public void test()
   {
      double initialHeight = 3.0;
      double startX = -0.1;
      double[] stepHeights = new double[] {0.1, 0.2, 0.3};
      double[] stepTreads = new double[] {0.4, 0.5};
      ListOfHeightsStairGroundProfile groundProfile = new ListOfHeightsStairGroundProfile(stepHeights, stepTreads, initialHeight, startX);

      double epsilonX = 1e-3;
      double epsilonZ = 1e-14;

      int nSteps = stepHeights.length;
      
      double currentX = startX;
      double currentHeight = initialHeight;
      for (int i = 0; i < nSteps; i++)
      {
         assertEquals(currentHeight, groundProfile.heightAt(currentX - epsilonX, 0.0, 0.0), epsilonZ);
         currentHeight += stepHeights[i];
         assertEquals(currentHeight, groundProfile.heightAt(currentX + epsilonX, 0.0, 0.0), epsilonZ);
         if (i < stepTreads.length)
            currentX += stepTreads[i];
      }

      System.out.println(currentX);
   }

}
