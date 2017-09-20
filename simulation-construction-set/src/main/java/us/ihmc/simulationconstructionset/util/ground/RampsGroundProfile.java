package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class RampsGroundProfile extends GroundProfileFromHeightMap
{
   private final double rampSlope;
   private final double rampLength;
   private final double flatgroundLengthAtZero;
   
   private final BoundingBox3D boundingBox;
   
   public RampsGroundProfile(double rampSlope, double rampLength, double flatgroundLengthAtZero)
   {
      this.rampSlope = rampSlope;
      this.rampLength = rampLength;
      this.flatgroundLengthAtZero = flatgroundLengthAtZero;
      
      boundingBox = new BoundingBox3D(new Point3D(-20.0, -20.0, Double.NEGATIVE_INFINITY), new Point3D(20.0, 20.0, Double.POSITIVE_INFINITY));
   }
   
   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      if (Math.abs(x) < flatgroundLengthAtZero / 2.0)
      {
         if (normalToPack != null)
         {
            normalToPack.setX(0.0);
            normalToPack.setY(0.0);
            normalToPack.setZ(1.0);
         }
         
         return 0.0;
      }
      else
      {
         x = (Math.abs(x) - (flatgroundLengthAtZero / 2.0)) % (2.0 * rampLength);
         
         if (x > rampLength)
         {
            x -= rampLength;
            
            if (normalToPack != null)
            {
               normalToPack.setX(Math.tan(rampSlope));
               normalToPack.setY(0.0);
               normalToPack.setZ(1.0);
               normalToPack.normalize();
            }
            
            return (rampSlope * rampLength) - (rampSlope * x);
         }
         else
         {
            if (normalToPack != null)
            {
               normalToPack.setX(-Math.tan(rampSlope));
               normalToPack.setY(0.0);
               normalToPack.setZ(1.0);
               normalToPack.normalize();
            }
            
            return rampSlope * x;
         }
      }
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {
      return heightAndNormalAt(x, y, z, null);
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }
}
