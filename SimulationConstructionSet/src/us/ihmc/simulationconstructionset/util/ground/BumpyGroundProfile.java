package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.BoundingBox3d;


public class BumpyGroundProfile extends GroundProfileFromHeightMap
{
   private static final double xMinDefault = -10.0, xMaxDefault = 10.0, yMinDefault = -10.0, yMaxDefault = 10.0;
   private static final double xAmp1Default = 0.2, xFreq1Default = 0.1, xAmp2Default = 0.1, xFreq2Default = 0.5;
   private static final double yAmp1Default = 0.1, yFreq1Default = 0.07, yAmp2Default = 0.05, yFreq2Default = 0.37;
   
   private final BoundingBox3d boundingBox;
   
   private final double xAmp1, xFreq1, xAmp2, xFreq2;
   private final double yAmp1, yFreq1, yAmp2, yFreq2;
   
   private final double flatgroundBoxWidthAtZero;

   public BumpyGroundProfile()
   {
      this(xAmp1Default, xFreq1Default, xAmp2Default, xFreq2Default, yAmp1Default, yFreq1Default, yAmp2Default, yFreq2Default);
   }
   
   public BumpyGroundProfile(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2, double flatgroundBoxWidthAtZero)
   {
      this(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2,
            xMinDefault, xMaxDefault, yMinDefault, yMaxDefault, flatgroundBoxWidthAtZero);
   }

   public BumpyGroundProfile(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2)
   {
      this(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2,
            xMinDefault, xMaxDefault, yMinDefault, yMaxDefault);
   }
   
   public BumpyGroundProfile(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2, double xMin,
         double xMax, double yMin, double yMax)
   {
      this(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2, xMin, xMax, yMin, yMax, 0.0);
   }

   public BumpyGroundProfile(double xAmp1, double xFreq1, double xAmp2, double xFreq2, double yAmp1, double yFreq1, double yAmp2, double yFreq2,
                             double xMin, double xMax, double yMin, double yMax, double flatgroundBoxWidthAtZero)
   {
      this.xAmp1 = xAmp1;
      this.xFreq1 = xFreq1;
      this.xAmp2 = xAmp2;
      this.xFreq2 = xFreq2;

      this.yAmp1 = yAmp1;
      this.yFreq1 = yFreq1;
      this.yAmp2 = yAmp2;
      this.yFreq2 = yFreq2;
      
      this.flatgroundBoxWidthAtZero = flatgroundBoxWidthAtZero;
      
      double zMax = Math.abs(xAmp1) + Math.abs(xAmp2) + Math.abs(yAmp1) + Math.abs(yAmp2);
      double zMin = -zMax;
      
      this.boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      if (Math.abs(x) < flatgroundBoxWidthAtZero / 2.0 && Math.abs(y) < flatgroundBoxWidthAtZero / 2.0)
      {
         if (normalToPack != null)
         {
            normalToPack.setX(0.0);
            normalToPack.setY(0.0);
            normalToPack.setZ(1.0);
         }
         
         return 0.0;
      }
      
      
      double height = (xAmp1 * Math.sin(2.0 * Math.PI * xFreq1 * x) + xAmp2 * Math.sin(2.0 * Math.PI * xFreq2 * x))
            + (yAmp1 * Math.sin(2.0 * Math.PI * yFreq1 * y) + yAmp2 * Math.sin(2.0 * Math.PI * yFreq2 * y));

      if (normalToPack != null)
      {
         double dzdx = xAmp1 * 2.0 * Math.PI * xFreq1 * Math.cos(2.0 * Math.PI * xFreq1 * x)
               + xAmp2 * 2.0 * Math.PI * xFreq2 * Math.cos(2.0 * Math.PI * xFreq2 * x);
         double dzdy = yAmp1 * 2.0 * Math.PI * yFreq1 * Math.cos(2.0 * Math.PI * yFreq1 * y)
               + yAmp2 * 2.0 * Math.PI * yFreq2 * Math.cos(2.0 * Math.PI * yFreq2 * y);
         normalToPack.setX(-dzdx);
         normalToPack.setY(-dzdy);
         normalToPack.setZ(1.0);

         normalToPack.normalize();
      }

      return height;
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {
      return heightAndNormalAt(x, y, z, null);
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }
}
