package us.ihmc.tools.exceptions;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class NoConvergenceExceptionTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCreateAndThrowSomeNoConvergenceExceptions()
   {
      int iter = 0;
      
      try
      {
         throw new NoConvergenceException();
      }
      catch (NoConvergenceException e)
      {
         iter = e.getIter();
      }
      
      assertTrue("Iter not equal -1", iter == -1);
      
      try
      {
         throw new NoConvergenceException(5);
      }
      catch (NoConvergenceException e)
      {
         iter = e.getIter();
      }
      
      assertTrue("Iter not equal -1", iter == 5);
   }
}
