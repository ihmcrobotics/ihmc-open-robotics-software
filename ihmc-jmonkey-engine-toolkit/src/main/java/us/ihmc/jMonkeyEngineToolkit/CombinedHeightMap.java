package us.ihmc.jMonkeyEngineToolkit;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.graphicsDescription.HeightMap;

public class CombinedHeightMap implements HeightMap
{
   private final ArrayList<HeightMap> heightMaps = new ArrayList<HeightMap>();
   private BoundingBox3D boundingBox = null;

   public void addHeightMap(HeightMap heightMap)
   {
      this.heightMaps.add(heightMap);

      if (boundingBox == null)
         boundingBox = heightMap.getBoundingBox();
      else
         boundingBox = BoundingBox3D.union(boundingBox, heightMap.getBoundingBox());
   }

   public double heightAt(double x, double y, double z)
   {
      Double heightAt = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < heightMaps.size(); i++)
      {
         HeightMap heightMap = heightMaps.get(i);
         if (heightMap.getBoundingBox().isXYInsideInclusive(x, y))
         {
            double localHeightAt = heightMap.heightAt(x, y, z);
            if (localHeightAt > heightAt)
               heightAt = localHeightAt;
         }
      }

      return heightAt;
   }

   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

}
