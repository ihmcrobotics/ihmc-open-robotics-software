package us.ihmc.exampleSimulations.singgleLeggedRobot;

public class MathForML
{
   public MathForML()
   {

   }

   public long factorial(int n)
   {
      long fact = 1;
      for (int i = 2; i <= n; i++)
      {
         fact = fact * i;
      }
      return fact;
   }
}
