package us.ihmc.quadrupedRobotics.output;

public class QuadrupedJointGainScheduler
{
   enum QuadrupedJointGains
   {
      NONE(0.0, 0.0),
      SOFT(100.0, 1.0),
      MEDIUM1(500.0, 50.0),
      MEDIUM2(1000.0, 50.0),
      HARD1(5000.0, 50.0),
      HARD(10000.0, 100.0);

      private final double stiffness;
      private final double damping;

      private static final QuadrupedJointGains[] values = values();

      QuadrupedJointGains(double stiffness, double damping)
      {
         this.stiffness = stiffness;
         this.damping = damping;
      }

      public double stiffness()
      {
         return stiffness;
      }

      public double damping()
      {
         return damping;
      }
   }

   public static QuadrupedJointGains getCorrespondingJointGains(double stiffness)
   {
      for (int i = 0; i < QuadrupedJointGains.values.length; i++)
      {
         if (stiffness <= QuadrupedJointGains.values[i].stiffness)
         {
            if (i > 0)
               return QuadrupedJointGains.values[i - 1];
            else
               return QuadrupedJointGains.values[i];
         }
      }

      return QuadrupedJointGains.values[QuadrupedJointGains.values.length - 1];
   }
}
