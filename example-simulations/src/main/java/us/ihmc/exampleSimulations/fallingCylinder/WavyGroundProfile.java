package us.ihmc.exampleSimulations.fallingCylinder;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;

public class WavyGroundProfile implements GroundProfile3D, HeightMapWithNormals
{
   private double xMin = -2.0, xMax = 2.0, yMin = -2.0, yMax = 2.0, zMin = -10.0, zMax = 10.0;
   
   private BoundingBox3D boundingBox = new BoundingBox3D(new Point3D(xMin, yMin, zMin), new Point3D(xMax, yMax, zMax));

   public WavyGroundProfile()
   {
   }
   
   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      return heightAt;
   }
   
   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      // TODO Auto-generated method stub
      return this;
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   

   @Override
   public BoundingBox3D getBoundingBox()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      // TODO Auto-generated method stub
      return false;
   }

  
}
