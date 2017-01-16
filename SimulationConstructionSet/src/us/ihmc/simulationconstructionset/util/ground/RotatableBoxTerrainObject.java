package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.HeightMapWithNormals;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;


public class RotatableBoxTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   protected final BoundingBox3d boundingBox;
   protected final Box3d box;
   public AppearanceDefinition appearance;
   private final Box3d.FaceName[] faceNames = Box3d.FaceName.values();
   protected Graphics3DObject linkGraphics;

   private final Point3d tempPoint = new Point3d();
   private final Vector3d zVector = new Vector3d(0.0, 0.0, 1.0);
   private final Point3d intersection = new Point3d();

   public RotatableBoxTerrainObject(Box3d box, AppearanceDefinition appearance)
   {
      this.box = box;
      this.appearance = appearance;
      
      Point3d[] vertices = box.getVertices();
      Point3d minPoint = new Point3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point3d maxPoint = new Point3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
      
      for (Point3d cornerPoint : vertices)
      {
         for (Direction direction : Direction.values())
         {
            double coordinate = MathTools.get(cornerPoint, direction);
            if (coordinate > MathTools.get(maxPoint, direction))
               MathTools.set(maxPoint, direction, coordinate);
            if (coordinate < MathTools.get(minPoint, direction))
               MathTools.set(minPoint, direction, coordinate);
         }
      }

//      minPoint.set(-1.0, -1.0, -1.0);
//      maxPoint.set(1.0, 1.0, 10.0);

      boundingBox = new BoundingBox3d(minPoint, maxPoint);

      addGraphics();
   }

   public RotatableBoxTerrainObject(RigidBodyTransform configuration, double lengthX, double widthY, double heightZ, AppearanceDefinition appearance)
   {
      this(new Box3d(configuration, lengthX, widthY, heightZ), appearance);
   }

   protected void addGraphics()
   {
      RigidBodyTransform transformCenterConventionToBottomConvention = box.getTransformCopy();

      transformCenterConventionToBottomConvention = TransformTools.transformLocalZ(transformCenterConventionToBottomConvention, -box.getDimension(Direction.Z) / 2.0);
      linkGraphics = new Graphics3DObject();
      linkGraphics.transform(transformCenterConventionToBottomConvention);

      linkGraphics.addCube(box.getDimension(Direction.X), box.getDimension(Direction.Y), box.getDimension(Direction.Z), appearance);
   }


   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;

   }

   public BoundingBox3d getBoundingBox()
   {
      return boundingBox;
   }

   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      return heightAt;
   }
   
   public double heightAt(double x, double y, double z)
   {
      // TODO: inefficient, magic epsilon
      tempPoint.set(x, y, z);
      double maxHeight = Double.NEGATIVE_INFINITY;
      for (Box3d.FaceName faceName : faceNames)
      {
         Plane3d face = box.getFace(faceName);
         face.getIntersectionWithLine(intersection, tempPoint, zVector);

         if (MathTools.isFinite(intersection))
         {
//            // check if point in range
            if (box.isInsideOrOnSurface(intersection, 1e-9) && intersection.getZ() > maxHeight)
            {
               maxHeight = intersection.getZ();
            }
         }
      }

      return maxHeight;
   }

   public double getXMin()
   {
      return boundingBox.getXMin();
   }

   public double getYMin()
   {
      return boundingBox.getYMin();
   }

   public double getXMax()
   {
      return boundingBox.getXMax();
   }

   public double getYMax()
   {
      return boundingBox.getYMax();
   }

   public double getZMin() { return boundingBox.getZMin();}

   public double getZMax() { return boundingBox.getZMax();}

   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInside(x, y);
   }

   public void closestIntersectionTo(double x, double y, double z, Point3d intersectionToPack)
   {
      tempPoint.set(x, y, z);
      box.checkIfInside(tempPoint, intersectionToPack, null);
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      tempPoint.set(x, y, z);
      box.checkIfInside(tempPoint, null, normalToPack);
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      tempPoint.set(x, y, z);
      box.checkIfInside(tempPoint, intersectionToPack, normalToPack);
   }
   
   public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
   {
      tempPoint.set(x, y, z);
      
      if (!box.isInsideOrOnSurface(tempPoint)) return false;

      box.checkIfInside(tempPoint, intersectionToPack, normalToPack);

      return true;
   }
   
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
}
