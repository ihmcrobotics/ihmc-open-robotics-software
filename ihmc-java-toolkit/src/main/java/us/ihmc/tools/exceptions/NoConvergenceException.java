package us.ihmc.tools.exceptions;

public class NoConvergenceException extends Exception
{
   private int iter;
   private static final long serialVersionUID = 6212719102444264616L;

   public NoConvergenceException()
   {
      this(-1);
   }

   public NoConvergenceException(int iter)
   {
      this.iter = iter;
   }

   public int getIter()
   {
      return iter;
   }
}
