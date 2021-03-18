package us.ihmc.quadrupedPlanning;

public enum QuadrupedGait
{
   PACE(0.0), AMBLE(90.0), TROT(180.0);

   private final double endPhaseShift;

   private QuadrupedGait(double endPhaseShift)
   {
      this.endPhaseShift = endPhaseShift;
   }

   public double getEndPhaseShift()
   {
      return endPhaseShift;
   }

   public static QuadrupedGait[] values = values();
}
