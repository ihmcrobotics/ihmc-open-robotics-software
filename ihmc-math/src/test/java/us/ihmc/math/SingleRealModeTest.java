package us.ihmc.math;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class SingleRealModeTest
{
   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

	@Test
   public void testDifferentLengths()
   {
      double eigenvalue = -2.0;
      double[] leftEigenvectorV = new double[] {1.0, 2.0};
      double[] rightEigenvectorW = new double[] {1.0};

      try
      {
         @SuppressWarnings("unused") SingleRealMode singleRealMode = new SingleRealMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
         fail();
      }
      catch (Exception e)
      {
      }
   }

	@Test
   public void testDotEqualsOne()
   {
      double eigenvalue = -2.0;
      double[] leftEigenvectorV = new double[] {1.0, 2.0};
      double[] rightEigenvectorW = new double[] {1.0, 3.0};

      try
      {
         @SuppressWarnings("unused")
         SingleRealMode singleRealMode = new SingleRealMode(eigenvalue, leftEigenvectorV, rightEigenvectorW);
         fail();
      }
      catch (Exception e)
      {
      }
   }
}
