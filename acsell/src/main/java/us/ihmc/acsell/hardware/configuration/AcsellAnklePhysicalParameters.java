package us.ihmc.acsell.hardware.configuration;

public class AcsellAnklePhysicalParameters
{
   
   private final double[] P0;
   private final double[] R0;
   private final double Kz;
   private final double Lr;
   
   public AcsellAnklePhysicalParameters(double[] P0, double[] R0, double Kz, double Lr)
   {
      this.P0 = P0;
      this.R0 = R0;
      this.Kz = Kz;
      this.Lr = Lr;
   }   
   
   public double[] getP0()
   {
      return P0;
   }
   public double[] getR0()
   {
      return R0;
   }
   public double getKz()
   {
      return Kz;
   }
   public double getLr()
   {
      return Lr;
   }

}
