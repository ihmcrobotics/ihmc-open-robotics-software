package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class RampTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final double xMin, xMax, yMin, yMax;
   private final double xStart, xEnd;
   private final double height;

   private final BoundingBox3d boundingBox;
   
   private Graphics3DObject linkGraphics;

   public RampTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double height, AppearanceDefinition appearance)
   {
      this.xStart = xStart;
      this.xEnd = xEnd;
      this.height = height;

      xMin = Math.min(xStart, xEnd);
      xMax = Math.max(xStart, xEnd);

      yMin = Math.min(yStart, yEnd);
      yMax = Math.max(yStart, yEnd);

      linkGraphics = new Graphics3DObject();
      linkGraphics.translate((xStart + xEnd) / 2.0, (yStart + yEnd) / 2.0, 0.0);

      if (xStart > xEnd)
         linkGraphics.rotate(Math.PI, Axis.Z);
      linkGraphics.addWedge(Math.abs(xEnd - xStart), Math.abs(yEnd - yStart), height, appearance);
      
      Point3d minPoint = new Point3d(xMin, yMin, Double.NEGATIVE_INFINITY);
      Point3d maxPoint = new Point3d(xMax, yMax, height);
      
      boundingBox = new BoundingBox3d(minPoint, maxPoint);
   }

   public RampTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double height)
   {
      this(xStart, yStart, xEnd, yEnd, height, YoAppearance.Black());
   }

   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      return heightAt;
   }

   public double heightAt(double x, double y, double z)
   {
      if ((x > xMin) && (x < xMax) && (y > yMin) && (y < yMax))
      {
         return (x - xStart) / (xEnd - xStart) * height;
      }

      return 0.0;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
   {
      double threshhold = 0.015;
      normal.x = 0.0;
      normal.y = 0.0;
      normal.z = 1.0;

      if ((x < xMin) || (x > xMax) || (y < yMin) || (y > yMax) || (z > height))
         return;

         /*
          * if (Math.abs(x-xMin) < threshhold)
          * {
          *   normal.x = -1.0;normal.y = 0.0;normal.z = 0.0;
          * }
          */

      else if (z > heightAt(x, y, z) - threshhold)
      {
         normal.x = height;
         normal.y = 0.0;
         normal.z = xStart - xEnd;

         normal.normalize();
         if (normal.z < 0.0)
            normal.scale(-1.0);
      }

      else if (Math.abs(x - xEnd) < threshhold)
      {
         if (xEnd > xStart)
            normal.x = 1.0;
         else
            normal.x = -1.0;
         normal.y = 0.0;
         normal.z = 0.0;
      }

      else if (Math.abs(y - yMin) < threshhold)
      {
         normal.x = 0.0;
         normal.y = -1.0;
         normal.z = 0.0;
      }

      else if (Math.abs(y - yMax) < threshhold)
      {
         normal.x = 0.0;
         normal.y = 1.0;
         normal.z = 0.0;
      }
   }


   public void closestIntersectionTo(double x, double y, double z, Point3d intersection)
   {
      intersection.x = x;    // Go Straight Up for now...
      intersection.y = y;
      intersection.z = heightAt(x, y, z);
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersection, Vector3d normal)
   {
      intersection.x = x;    // Go Straight Up for now...
      intersection.y = y;
      intersection.z = heightAt(x, y, z);

      surfaceNormalAt(x, y, z, normal);
   }

   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      if (z > heightAt) return false;

      intersectionToPack.set(x, y, heightAt); 
      surfaceNormalAt(x, y, z, normalToPack);
      
      return true;
   }
   
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInside(x, y);
   }

   public double getXMin()
   {
      return xMin;
   }

   public double getYMin()
   {
      return yMin;
   }

   public double getXMax()
   {
      return xMax;
   }

   public double getYMax()
   {
      return yMax;
   }

   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }
   
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

}
