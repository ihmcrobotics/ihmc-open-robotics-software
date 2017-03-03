package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class SimpleTableTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   public static final double DEFAULT_TABLE_LENGTH = 2.0;
   private double TABLE_LENGTH, TABLE_THICKNESS, TABLE_WIDTH;

   private final BoundingBox3d boundingBox;

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

      boundingBox = new BoundingBox3d(minPoint, maxPoint);

      linkGraphics = new Graphics3DObject();

      linkGraphics.translate((xStart + xEnd) / 2.0, (yStart + yEnd) / 2.0, zMin + TABLE_THICKNESS / 2);
      linkGraphics.scale(new Vector3D(TABLE_LENGTH, TABLE_WIDTH, TABLE_THICKNESS));
      linkGraphics.addModelFile("models/plasticTableTop.obj");

      if (TABLE_LENGTH < TABLE_WIDTH)
         linkGraphics.rotate(Math.PI / 2, Axis.Z);
      linkGraphics.scale(new Vector3D(1, 1, boundingBox.getZMax() / TABLE_THICKNESS));
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
      if ((x > boundingBox.getXMin()) && (x < boundingBox.getXMax()) && (y > boundingBox.getYMin()) && (y < boundingBox.getYMax()))
      {
         return boundingBox.getZMax();
      }

      return 0.0;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if ((z > (boundingBox.getZMax() - TABLE_THICKNESS)) && (z < boundingBox.getZMax()))
         return boundingBox.isInside(x, y, z);

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

      if (!boundingBox.isXYInside(x, y) || (z > boundingBox.getZMax() - threshhold))
         return;

      if (Math.abs(x - boundingBox.getXMin()) < threshhold)
      {
         normal.setX(-1.0);
         normal.setY(0.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(x - boundingBox.getXMax()) < threshhold)
      {
         normal.setX(1.0);
         normal.setY(0.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(y - boundingBox.getYMin()) < threshhold)
      {
         normal.setX(0.0);
         normal.setY(-1.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(y - boundingBox.getYMax()) < threshhold)
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
      return boundingBox.getXMin();
   }

   public double getXMax()
   {
      return boundingBox.getXMax();
   }

   public double getYMin()
   {
      return boundingBox.getYMin();
   }

   public double getYMax()
   {
      return boundingBox.getYMax();
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
}
