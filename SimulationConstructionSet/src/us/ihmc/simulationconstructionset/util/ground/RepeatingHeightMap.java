package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Vector3d;

import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class RepeatingHeightMap implements HeightMapWithNormals
{
   private double xMin, xMax, yMin, yMax;
   private double xDistance, yDistance;
   
   private final HeightMapWithNormals heightMap;
   private final BoundingBox3d boundingBox;

   public RepeatingHeightMap(HeightMapWithNormals heightMap, double xMin, double xMax, double yMin, double yMax)
   {
     this.xMin = xMin;
     this.xMax = xMax;
     
     this.yMin = yMin;
     this.yMax = yMax;
     
     this.xDistance = this.xMax - this.xMin;
     this.yDistance = this.yMax - this.yMin;
     
     this.heightMap = heightMap;

     double zMin = heightMap.getBoundingBox().getZMin();
     double zMax = heightMap.getBoundingBox().getZMax();
     this.boundingBox = new BoundingBox3d(xMin, yMin, zMin, xMax, yMax, zMax);
   }
   
   private double xLocal(double xGlobal)
   {
     return (Math.abs(xGlobal - xMin) % xDistance) + xMin;
   }
   
   private double yLocal(double yGlobal)
   {
      return (Math.abs(yGlobal - yMin) % yDistance) + yMin;
   }
   
   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double localX = xLocal(x);
      double localY = yLocal(y);
      
      return heightMap.heightAndNormalAt(localX, localY, z, normalToPack);
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {      
      return heightMap.heightAt(xLocal(x), yLocal(y), z);
   }
   
   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }
   
}
