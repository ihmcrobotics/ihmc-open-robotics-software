package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.BoundingBox3d;

public abstract class GroundProfileFromHeightMap implements HeightMapWithNormals, GroundProfile3D
{   
   public GroundProfileFromHeightMap()
   {
   }

   public boolean isClose(double x, double y, double z)
   {
      return this.getBoundingBox().isInside(x, y, z);
   }

   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      double heightAt = this.heightAndNormalAt(x, y, z, normalToPack);
      intersectionToPack.set(x, y, heightAt);
      
      return (z < heightAt);
   }
   
   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      double heightAt = this.heightAndNormalAt(x, y, z, normalToPack);
      intersectionToPack.set(x, y, heightAt);
   }
   
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
         public BoundingBox3d getBoundingBox()
         {
            return heightMap.getBoundingBox();
         }

         @Override
         public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
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
         public BoundingBox3d getBoundingBox()
         {
            return heightMap.getBoundingBox();
         }

         @Override
         public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
         {
            return heightMap.heightAndNormalAt(x, y, z, normalToPack);
         }
      };

      return ret;
   }
   
   public static GroundProfileFromHeightMap createAGroundProfileFromAHeightMapWithPoints(final HeightMapWithPoints heightMapWithPoints, final BoundingBox3d boundingBox)
   {
      GroundProfileFromHeightMap ret = new GroundProfileFromHeightMap()
      {

         @Override
         public double heightAt(double x, double y, double z)
         {
            return heightMapWithPoints.getHeightAtPoint(x, y);
         }

         @Override
         public BoundingBox3d getBoundingBox()
         {
            return boundingBox;
         }

         @Override
         public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
         {
            normalToPack.set(0.0, 0.0, 1.0);
            return heightMapWithPoints.getHeightAtPoint(x, y);
         }
      };

      return ret;
   }

}
