package us.ihmc.exampleSimulations.planarWalker;

public class HipAngleCapturePointCalculator
{
   private static double g = 9.81;
   
   public static double getHipAngle(double bodyVelocity, double legLength)
   {
      double miniumumValue = Double.POSITIVE_INFINITY;
      double bestAngle = 0.0;
      for(double angle=0.0; angle<Math.PI/2.0; angle = angle + 0.001)
      {
         double term1 = g * legLength * legLength * (Math.sin(angle) * Math.sin(angle));
         double term2 = bodyVelocity * bodyVelocity * legLength * Math.cos(angle);
         
         double value = term1 - term2;
         //System.out.println("angle=" + angle + ", value=" + value);
         if (Math.abs(value) < Math.abs(miniumumValue))
         {
            miniumumValue = value;
            bestAngle = angle;
         }
      }
      
      return bestAngle;
   }
   
   public static void main(String[] args)
   {
      double angle = HipAngleCapturePointCalculator.getHipAngle(0.5, 0.8);
      System.out.println("***********");
      System.out.println("angle=" + angle);
   }
}
