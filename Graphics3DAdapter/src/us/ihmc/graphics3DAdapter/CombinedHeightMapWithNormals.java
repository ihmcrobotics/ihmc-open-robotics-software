package us.ihmc.graphics3DAdapter;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class CombinedHeightMapWithNormals implements HeightMapWithNormals
{
   private final ArrayList<HeightMapWithNormals> heightMaps = new ArrayList<HeightMapWithNormals>();
   private BoundingBox3d boundingBox = null;

   public void addHeightMap(HeightMapWithNormals heightMap)
   {
      this.heightMaps.add(heightMap);

      if (boundingBox == null)
         boundingBox = heightMap.getBoundingBox();
      else
         boundingBox = BoundingBox3d.union(boundingBox, heightMap.getBoundingBox());
   }


   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      Double heightAt = Double.NEGATIVE_INFINITY;
      normalToPack.set(0.0, 0.0, 1.0);

      for (int i = 0; i < heightMaps.size(); i++)
      {
         HeightMapWithNormals heightMap = heightMaps.get(i);
         if (heightMap.getBoundingBox().isXYInside(x, y))
         {
            double localHeightAt = heightMap.heightAt(x, y, z);
            if (localHeightAt > heightAt)
            {
               heightAt = localHeightAt;
               heightMap.heightAndNormalAt(x, y, z, normalToPack);
            }
         }
      }

      return heightAt;
   }

   public double heightAt(double x, double y, double z)
   {
      Double heightAt = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < heightMaps.size(); i++)
      {
         HeightMap heightMap = heightMaps.get(i);
         if (heightMap.getBoundingBox().isXYInside(x, y))
         {
            double localHeightAt = heightMap.heightAt(x, y, z);
            if (localHeightAt > heightAt)
               heightAt = localHeightAt;
         }
      }

      return heightAt;
   }

   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

}
