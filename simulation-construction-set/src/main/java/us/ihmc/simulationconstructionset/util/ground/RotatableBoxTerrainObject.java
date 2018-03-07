package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.geometry.Shape3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;

public class RotatableBoxTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   protected final BoundingBox3D boundingBox = new BoundingBox3D();
   protected final Box3D box;
   public AppearanceDefinition appearance;
   protected Graphics3DObject linkGraphics;

   private final Point3D tempPoint = new Point3D();
   private final Vector3D zVector = new Vector3D(0.0, 0.0, 1.0);

   private final Point3D intersectionA = new Point3D();
   private final Point3D intersectionB = new Point3D();

   private ArrayList<Shape3D> simpleShapes = new ArrayList<>();
   
   public RotatableBoxTerrainObject(Box3D box, AppearanceDefinition appearance)
   {
      this.box = box;
      this.appearance = appearance;

      box.getBoundingBox3D(boundingBox);

      addGraphics();
      
      simpleShapes.add(this.box);
   }

   public RotatableBoxTerrainObject(RigidBodyTransform configuration, double lengthX, double widthY, double heightZ, AppearanceDefinition appearance)
   {
      this(new Box3D(configuration, lengthX, widthY, heightZ), appearance);
   }

   protected void addGraphics()
   {
      RigidBodyTransform transformCenterConventionToBottomConvention = new RigidBodyTransform();
      box.getPose(transformCenterConventionToBottomConvention);

      transformCenterConventionToBottomConvention.appendTranslation(0.0, 0.0, -box.getSizeZ() / 2.0);
      linkGraphics = new Graphics3DObject();
      linkGraphics.transform(transformCenterConventionToBottomConvention);

      linkGraphics.addCube(box.getSizeX(), box.getSizeY(), box.getSizeZ(), appearance);
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
      zVector.set(0.0, 0.0, 1.0);
      int numberOfIntersections = box.intersectionWith(tempPoint, zVector, intersectionA, intersectionB);

      if (numberOfIntersections == 0)
         return Double.NEGATIVE_INFINITY;
      else if (numberOfIntersections == 1)
         return intersectionA.getZ();
      else
         return Math.max(intersectionA.getZ(), intersectionB.getZ());
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

   public double getZMin()
   {
      return boundingBox.getMinZ();
   }

   public double getZMax()
   {
      return boundingBox.getMaxZ();
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInsideInclusive(x, y);
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

      if (!box.isInsideOrOnSurface(tempPoint))
         return false;

      box.checkIfInside(tempPoint, intersectionToPack, normalToPack);

      return true;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
   
   @Override
   public List<? extends Shape3D> getSimpleShapes()
   {
      return simpleShapes;
   }
}
