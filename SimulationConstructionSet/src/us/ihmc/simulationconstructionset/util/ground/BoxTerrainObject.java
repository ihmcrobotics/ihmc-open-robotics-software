package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;


public class BoxTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final BoundingBox3D boundingBox;
   private Graphics3DObject linkGraphics;


   public BoxTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double zStart, double zEnd, AppearanceDefinition appearance)
   {
     double xMin = Math.min(xStart, xEnd);
     double xMax = Math.max(xStart, xEnd);

     double yMin = Math.min(yStart, yEnd);
     double yMax = Math.max(yStart, yEnd);

     double zMin = Math.min(zStart, zEnd);
     double zMax = Math.max(zStart, zEnd);
     
     Point3D minPoint = new Point3D(xMin, yMin, zMin);
     Point3D maxPoint = new Point3D(xMax, yMax, zMax);
     
     boundingBox = new BoundingBox3D(minPoint, maxPoint);
     
     linkGraphics = new Graphics3DObject();
          
     linkGraphics.translate((xStart+xEnd)/2.0,(yStart+yEnd)/2.0, zMin);

     linkGraphics.addCube(Math.abs(xEnd-xStart), Math.abs(yEnd-yStart), zMax-zMin, appearance);
 }

 public BoxTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double height, AppearanceDefinition appearance)
 {
   this(xStart, yStart, xEnd, yEnd, 0.0, height, appearance);
 }

 public BoxTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double height)
 {
   this(xStart, yStart, xEnd, yEnd, height, YoAppearance.Gray());
 }

 public BoxTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double zStart, double zEnd)
 {
   this(xStart, yStart, xEnd, yEnd, zStart, zEnd, YoAppearance.Gray());
 }


 @Override
public Graphics3DObject getLinkGraphics()
 {
   return linkGraphics;

 }

 @Override
public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
 {
    double heightAt = this.heightAt(x, y, z);
    this.surfaceNormalAt(x, y, z, normalToPack);
    
    return heightAt;
 }
 
 @Override
public double heightAt(double x, double y, double z)
 {
   if ((x > boundingBox.getMinX()) && (x < boundingBox.getMaxX()) && (y > boundingBox.getMinY()) && (y < boundingBox.getMaxY()))
   {
     return boundingBox.getMaxZ();
   }

   return 0.0;
 }
 
 
   private void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      double threshhold = 0.015;
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);

      if (!boundingBox.isXYInsideInclusive(x, y) || (z > boundingBox.getMaxZ() - threshhold))
         return;

      if (Math.abs(x - boundingBox.getMinX()) < threshhold)
      {
         normal.setX(-1.0);
         normal.setY(0.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(x - boundingBox.getMaxX()) < threshhold)
      {
         normal.setX(1.0);
         normal.setY(0.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(y - boundingBox.getMinY()) < threshhold)
      {
         normal.setX(0.0);
         normal.setY(-1.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(y - boundingBox.getMaxY()) < threshhold)
      {
         normal.setX(0.0);
         normal.setY(1.0);
         normal.setZ(0.0);
      }
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersection, Vector3D normal)
   {
      intersection.setX(x);    // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));

      surfaceNormalAt(x, y, z, normal);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      intersectionToPack.setX(x);    // Go Straight Up for now...
      intersectionToPack.setY(y);
      intersectionToPack.setZ(heightAt(x, y, z));

      surfaceNormalAt(x, y, z, normalToPack);
      
      return (z < intersectionToPack.getZ());
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return (boundingBox.isXYInsideInclusive(x, y));
   }


   public double getXMin()
   {
      return boundingBox.getMinX();
   }

   public double getYMin()
   {
      return boundingBox.getMinY();
   }

   public double getXMax()
   {
      return boundingBox.getMaxX();
   }

   public double getYMax()
   {
      return boundingBox.getMaxY();
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

}
