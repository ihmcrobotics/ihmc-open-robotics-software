package us.ihmc.graphicsDescription.input.mouse;

public class Mouse3DPollData
{
   private double dx;
   private double dy;
   private double dz;
   private double drx;
   private double dry;
   private double drz;
   
   public Mouse3DPollData(double dx, double dy, double dz, double drx, double dry, double drz)
   {
      this.dx = dx;
      this.dy = dy;
      this.dz = dz;
      this.drx = drx;
      this.dry = dry;
      this.drz = drz;
   }

   public double getDx()
   {
      return dx;
   }

   public double getDy()
   {
      return dy;
   }

   public double getDz()
   {
      return dz;
   }

   public double getDrx()
   {
      return drx;
   }

   public double getDry()
   {
      return dry;
   }

   public double getDrz()
   {
      return drz;
   }
}
