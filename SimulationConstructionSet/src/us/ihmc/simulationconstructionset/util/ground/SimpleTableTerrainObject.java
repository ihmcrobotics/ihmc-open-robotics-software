package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.Axis;

public class SimpleTableTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   public static final double DEFAULT_TABLE_LENGTH = 2.0;
   private double TABLE_LENGTH, TABLE_THICKNESS, TABLE_WIDTH;

   private final BoundingBox3D boundingBox;

   private Graphics3DObject linkGraphics;

   public SimpleTableTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double zStart, double zEnd)
   {
      this.TABLE_LENGTH = Math.abs(xStart - xEnd);
      this.TABLE_WIDTH = Math.abs(yStart - yEnd);
      this.TABLE_THICKNESS = Math.abs(zStart - zEnd);

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

      linkGraphics.translate((xStart + xEnd) / 2.0, (yStart + yEnd) / 2.0, zMin + TABLE_THICKNESS / 2);
      linkGraphics.scale(new Vector3D(TABLE_LENGTH, TABLE_WIDTH, TABLE_THICKNESS));
      linkGraphics.addModelFile("models/plasticTableTop.obj");

      if (TABLE_LENGTH < TABLE_WIDTH)
         linkGraphics.rotate(Math.PI / 2, Axis.Z);
      linkGraphics.scale(new Vector3D(1, 1, boundingBox.getMaxZ() / TABLE_THICKNESS));
      linkGraphics.addModelFile("models/FoldingTableLegs.obj");
   }

   public double getWidth()
   {
      return this.TABLE_WIDTH;
   }

   public double getThickness()
   {
      return this.TABLE_THICKNESS;
   }

   public double getLength()
   {
      return this.TABLE_LENGTH;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, heightAt, normalToPack);
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

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if ((z > (boundingBox.getMaxZ() - TABLE_THICKNESS)) && (z < boundingBox.getMaxZ()))
         return boundingBox.isInsideInclusive(x, y, z);

      return false;
   }

   public void closestIntersectionTo(double x, double y, double z, Point3D intersection)
   {
      intersection.setX(x);
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
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

   public double getXMin()
   {
      return boundingBox.getMinX();
   }

   public double getXMax()
   {
      return boundingBox.getMaxX();
   }

   public double getYMin()
   {
      return boundingBox.getMinY();
   }

   public double getYMax()
   {
      return boundingBox.getMaxY();
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
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
