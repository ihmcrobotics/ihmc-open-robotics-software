package us.ihmc.tools.exceptions;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class NoConvergenceExceptionTest
{
   @Test
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
