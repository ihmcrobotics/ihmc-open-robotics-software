package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class RotatableRampTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final double xGlobalMin, xGlobalMax, yGlobalMin, yGlobalMax;
   private final double xLocalMin, xLocalMax, yLocalMin, yLocalMax;
   private final double height, run;

   private final BoundingBox3d boundingBox;

   private RigidBodyTransform transform;
   private RigidBodyTransform inverseTransform;
   
   Point3D pointToTransform=new Point3D();
   
   private Graphics3DObject linkGraphics;

   /**
    * @param xStartBeforeYaw
    * @param yStartBeforeYaw
    * @param xEndBeforeYaw
    * @param yEndBeforeYaw
    * @param height
    * @param degreesYaw
    * @param appearance
    */
   public RotatableRampTerrainObject(double xCenter, double yCenter, double run, double width, double height,
                                     double degreesYaw, AppearanceDefinition appearance)
   {
      boolean slopeDown = run<0;
      double radiansYaw = Math.toRadians(degreesYaw);
      transform = new RigidBodyTransform();
      RotationMatrix a1=new RotationMatrix();
      a1.setToYawMatrix(radiansYaw + (slopeDown?Math.PI:0));
      transform.setRotation(a1);
      transform.setTranslation(new Vector3D(xCenter,yCenter,0));
      inverseTransform = new RigidBodyTransform();
      inverseTransform.setAndInvert(transform);
      
      Point2D[] rampCorners = {
            transformToGlobalCoordinates(new Point2D(-run/2,width/2)),
            transformToGlobalCoordinates(new Point2D(run/2,  width/2)),
            transformToGlobalCoordinates(new Point2D(-run/2,-width/2)),
            transformToGlobalCoordinates(new Point2D(run/2,  -width/2))
      };

      this.height = height;
      double absRun=Math.abs(run);
      this.run = absRun;
      
      xLocalMin = -absRun/2;
      xLocalMax = absRun/2;

      yLocalMin = -width/2;
      yLocalMax = width/2;

      
      xGlobalMin =Math.min(Math.min(rampCorners[0].getX(),rampCorners[1].getX()), Math.min(rampCorners[2].getX(),rampCorners[3].getX()));
      xGlobalMax =Math.max(Math.max(rampCorners[0].getX(),rampCorners[1].getX()), Math.max(rampCorners[2].getX(),rampCorners[3].getX()));

      yGlobalMin = Math.min(Math.min(rampCorners[0].getY(),rampCorners[1].getY()), Math.min(rampCorners[2].getY(),rampCorners[3].getY()));
      yGlobalMax = Math.max(Math.max(rampCorners[0].getY(),rampCorners[1].getY()), Math.max(rampCorners[2].getY(),rampCorners[3].getY()));
      
      linkGraphics = new Graphics3DObject();
      linkGraphics.translate(xCenter, yCenter, 0.0);
      linkGraphics.rotate(radiansYaw +(slopeDown?Math.PI:0), Axis.Z);
      linkGraphics.addWedge(absRun, width, height, appearance);

      Point3D minPoint = new Point3D(xGlobalMin, yGlobalMin, Double.NEGATIVE_INFINITY);
      Point3D maxPoint = new Point3D(xGlobalMax, yGlobalMax, height);

      boundingBox = new BoundingBox3d(minPoint, maxPoint);
   }

   public RotatableRampTerrainObject(double xCenter, double yCenter, double run, double width, double height,
         double degreesYaw)
     {
        this( xCenter,  yCenter,  run,  width,  height,
             degreesYaw, YoAppearance.Black());
     }

   private Point2D transformToGlobalCoordinates(Point2D localCoordinate)
   {
      pointToTransform.setX(localCoordinate.getX());
      pointToTransform.setY(localCoordinate.getY());
      pointToTransform.setZ(0);
      transform.transform(pointToTransform);
      return new Point2D(pointToTransform.getX(),pointToTransform.getY());
   }

   private Point2D transformToLocalCoordinates(Point2D globalCoordinate)
   {
      pointToTransform.setX(globalCoordinate.getX());
      pointToTransform.setY(globalCoordinate.getY());
      pointToTransform.setZ(0);
      inverseTransform.transform(pointToTransform);
      return new Point2D(pointToTransform.getX(),pointToTransform.getY());
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, heightAt, normalToPack);
      return heightAt;
   }
   
   @Override
   public double heightAt(double xGlobal, double yGlobal, double zGlobal)
   {
      Point2D localPoint = transformToLocalCoordinates(new Point2D(xGlobal,yGlobal));
      double xLocal=localPoint.getX();
      double yLocal=localPoint.getY();

      if (localPointOnRamp(xLocal,yLocal))
      {
         return (xLocal-xLocalMin) / run * height;
      }


      return 0.0;
   }

   private boolean localPointOnRamp(double xLocal, double yLocal)
   {
      return (xLocal >= xLocalMin) && (xLocal <= xLocalMax) && (yLocal >= yLocalMin) && (yLocal <= yLocalMax);
   }

   public void surfaceNormalAt(double xGlobal, double yGlobal, double z, Vector3D normal)
   {
      Point2D localPoint = transformToLocalCoordinates(new Point2D(xGlobal,yGlobal));
      double xLocal=localPoint.getX();
      double yLocal=localPoint.getY();

      double threshhold = 0.015;
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);

      if (!localPointOnRamp(xLocal,yLocal))
         return;

         /*
          * if (Math.abs(x-xMin) < threshhold)
          * {
          *   normal.x = -1.0;normal.y = 0.0;normal.z = 0.0;
          * }
          */

      else if (z > heightAt(xGlobal, yGlobal, z) - threshhold)
      {
         normal.setX(height);
         normal.setY(0.0);
         normal.setZ(-run);

         normal.normalize();
         if (normal.getZ() < 0.0)
            normal.scale(-1.0);
      }

      else if (Math.abs(xLocal - xLocalMax) < threshhold)
      {
         if (xLocalMax > xLocalMin)
            normal.setX(1.0);
         else
            normal.setX(-1.0);
         normal.setY(0.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(yLocal - yLocalMin) < threshhold)
      {
         normal.setX(0.0);
         normal.setY(-1.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(yLocal - yLocalMax) < threshhold)
      {
         normal.setX(0.0);
         normal.setY(1.0);
         normal.setZ(0.0);
      }
      
      transform.transform(normal);
      
   }


   public void closestIntersectionTo(double x, double y, double z, Point3D intersection)
   {
      intersection.setX(x);    // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersection, Vector3D normal)
   {
      closestIntersectionTo(x, y, z, intersection);
      surfaceNormalAt(x, y, z, normal);
   }
   
   
   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      closestIntersectionTo(x, y, z, intersectionToPack);
      surfaceNormalAt(x, y, z, normalToPack);
      
      return (z < intersectionToPack.getZ());
   }


   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInside(x, y);
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
