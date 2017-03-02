package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.geometry.shapes.Plane3d;


public class RotatableBoxTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   protected final BoundingBox3d boundingBox;
   protected final Box3d box;
   public AppearanceDefinition appearance;
   private final Box3d.FaceName[] faceNames = Box3d.FaceName.values();
   protected Graphics3DObject linkGraphics;

   private final Point3D tempPoint = new Point3D();
   private final Vector3D zVector = new Vector3D(0.0, 0.0, 1.0);
   private final Point3D intersection = new Point3D();

   public RotatableBoxTerrainObject(Box3d box, AppearanceDefinition appearance)
   {
      this.box = box;
      this.appearance = appearance;
      
      Point3D[] vertices = box.getVertices();
      Point3D minPoint = new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point3D maxPoint = new Point3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
      
      for (Point3D cornerPoint : vertices)
      {
         for (Direction direction : Direction.values())
         {
            double coordinate = Direction.get(cornerPoint, direction);
            if (coordinate > Direction.get(maxPoint, direction))
               Direction.set(maxPoint, direction, coordinate);
            if (coordinate < Direction.get(minPoint, direction))
               Direction.set(minPoint, direction, coordinate);
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
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      return heightAt;
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {
      // TODO: inefficient, magic epsilon
      tempPoint.set(x, y, z);
      double maxHeight = Double.NEGATIVE_INFINITY;
      for (Box3d.FaceName faceName : faceNames)
      {
         Plane3d face = box.getFace(faceName);
         face.getIntersectionWithLine(intersection, tempPoint, zVector);

         if (EuclidCoreMissingTools.isFinite(intersection))
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

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInside(x, y);
   }

   public void closestIntersectionTo(double x, double y, double z, Point3D intersectionToPack)
   {
      tempPoint.set(x, y, z);
      box.checkIfInside(tempPoint, intersectionToPack, null);
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      tempPoint.set(x, y, z);
      box.checkIfInside(tempPoint, null, normalToPack);
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      tempPoint.set(x, y, z);
      box.checkIfInside(tempPoint, intersectionToPack, normalToPack);
   }
   
   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      tempPoint.set(x, y, z);
      
      if (!box.isInsideOrOnSurface(tempPoint)) return false;

      box.checkIfInside(tempPoint, intersectionToPack, normalToPack);

      return true;
   }
   
   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
}
