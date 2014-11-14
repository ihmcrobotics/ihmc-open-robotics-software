package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.HeightMapWithNormals;

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

}
