package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;

public abstract class GroundProfileFromHeightMap implements HeightMapWithNormals, GroundProfile3D
{   
   public GroundProfileFromHeightMap()
   {
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return this.getBoundingBox().isInsideInclusive(x, y, z);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      double heightAt = this.heightAndNormalAt(x, y, z, normalToPack);
      intersectionToPack.set(x, y, heightAt);
      
      return (z < heightAt);
   }
   
   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      double heightAt = this.heightAndNormalAt(x, y, z, normalToPack);
      intersectionToPack.set(x, y, heightAt);
   }
   
   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
   
   public static GroundProfileFromHeightMap createAGroundProfileFromAHeightMap(final HeightMap heightMap)
   {
      GroundProfileFromHeightMap ret = new GroundProfileFromHeightMap()
      {

         @Override
         public double heightAt(double x, double y, double z)
         {
            return heightMap.heightAt(x, y, z);
         }

         @Override
         public BoundingBox3D getBoundingBox()
         {
            return heightMap.getBoundingBox();
         }

         @Override
         public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
         {
            normalToPack.set(0.0, 0.0, 1.0);
            return heightMap.heightAt(x, y, z);
         }
      };

      return ret;
   }
   
   public static GroundProfileFromHeightMap createAGroundProfileFromAHeightMapWithNormals(final HeightMapWithNormals heightMap)
   {
      GroundProfileFromHeightMap ret = new GroundProfileFromHeightMap()
      {

         @Override
         public double heightAt(double x, double y, double z)
         {
            return heightMap.heightAt(x, y, z);
         }

         @Override
         public BoundingBox3D getBoundingBox()
         {
            return heightMap.getBoundingBox();
         }

         @Override
         public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
         {
            return heightMap.heightAndNormalAt(x, y, z, normalToPack);
         }
      };

      return ret;
   }
   
   public static GroundProfileFromHeightMap createAGroundProfileFromAHeightMapWithPoints(final HeightMapWithPoints heightMapWithPoints, final BoundingBox3D boundingBox)
   {
      GroundProfileFromHeightMap ret = new GroundProfileFromHeightMap()
      {

         @Override
         public double heightAt(double x, double y, double z)
         {
            return heightMapWithPoints.getHeightAtPoint(x, y);
         }

         @Override
         public BoundingBox3D getBoundingBox()
         {
            return boundingBox;
         }

         @Override
         public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
         {
            normalToPack.set(0.0, 0.0, 1.0);
            return heightMapWithPoints.getHeightAtPoint(x, y);
         }
      };

      return ret;
   }

}
