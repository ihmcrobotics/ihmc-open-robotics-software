package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.shapes.Box3d;

public class CombinedTerrainObject3D implements TerrainObject3D, HeightMapWithNormals
{
   private BoundingBox3D boundingBox = null;

   private ArrayList<TerrainObject3D> terrainObjects = new ArrayList<TerrainObject3D>();
   private Graphics3DObject linkGraphics;

   private final String name;

   private final Point3D tempPointToCheck = new Point3D();

   public CombinedTerrainObject3D(String name)
   {
      linkGraphics = new Graphics3DObject();
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public void addSphere(double xCenter, double yCenter, double zCenter, double radius, AppearanceDefinition appearance)
   {
      SphereTerrainObject sphere = new SphereTerrainObject(xCenter, yCenter, zCenter, radius, appearance);
      addTerrainObject(sphere);
   }

   public void addBox(double xStart, double yStart, double xEnd, double yEnd, double height, AppearanceDefinition appearance)
   {
      BoxTerrainObject box = new BoxTerrainObject(xStart, yStart, xEnd, yEnd, height, appearance);
      addTerrainObject(box);
   }

   public void addBox(double xStart, double yStart, double xEnd, double yEnd, double zStart, double zEnd)
   {
      BoxTerrainObject box = new BoxTerrainObject(xStart, yStart, xEnd, yEnd, zStart, zEnd);
      addTerrainObject(box);
   }

   public void addBox(double xStart, double yStart, double xEnd, double yEnd, double zStart, double zEnd, AppearanceDefinition appearance)
   {
      BoxTerrainObject box = new BoxTerrainObject(xStart, yStart, xEnd, yEnd, zStart, zEnd, appearance);
      addTerrainObject(box);
   }

   public void addRotatableBox(RigidBodyTransform configuration, double xLength, double yWidth, double zLength, AppearanceDefinition appearanceDefinition)
   {
      Box3d box3d = new Box3d(configuration, xLength, yWidth, zLength);
      RotatableBoxTerrainObject box = new RotatableBoxTerrainObject(box3d, appearanceDefinition);
      addTerrainObject(box);
   }

   public void addRotatableBox(Box3d box, AppearanceDefinition appearanceDefinition)
   {
      RotatableBoxTerrainObject terrainObject = new RotatableBoxTerrainObject(box, appearanceDefinition);
      addTerrainObject(terrainObject);
   }

   public void addBox(double xStart, double yStart, double xEnd, double yEnd, double height)
   {
      BoxTerrainObject box = new BoxTerrainObject(xStart, yStart, xEnd, yEnd, height);
      addTerrainObject(box);
   }

   public void addCylinder(RigidBodyTransform location, double height, double radius, AppearanceDefinition appearance)
   {
      CylinderTerrainObject cylinder = new CylinderTerrainObject(location, height, radius, appearance);
      addTerrainObject(cylinder);
   }

   public void addCone(double xMiddle, double yMiddle, double bottomRadius, double topRadius, double height, AppearanceDefinition appearance)
   {
      ConeTerrainObject cone = new ConeTerrainObject(xMiddle, yMiddle, bottomRadius, topRadius, height, appearance);
      addTerrainObject(cone);
   }

   public void addCone(double xMiddle, double yMiddle, double bottomRadius, double topRadius, double height)
   {
      ConeTerrainObject cone = new ConeTerrainObject(xMiddle, yMiddle, bottomRadius, topRadius, height);
      addTerrainObject(cone);
   }

   public void addRotatedRamp(double xCenter, double yCenter, double xRun, double yWidth, double height, double yawDegrees, AppearanceDefinition appearance)
   {
      RotatableRampTerrainObject ramp = new RotatableRampTerrainObject(xCenter, yCenter, xRun, yWidth, height, yawDegrees, appearance);
      addTerrainObject(ramp);
   }

   public void addRamp(double xStart, double yStart, double xEnd, double yEnd, double height, AppearanceDefinition appearance)
   {
      RampTerrainObject ramp = new RampTerrainObject(xStart, yStart, xEnd, yEnd, height, appearance);
      addTerrainObject(ramp);
   }

   public void addRamp(double xStart, double yStart, double xEnd, double yEnd, double height)
   {
      RampTerrainObject ramp = new RampTerrainObject(xStart, yStart, xEnd, yEnd, height);
      addTerrainObject(ramp);
   }

   public void addTable(double xStart, double yStart, double xEnd, double yEnd, double zStart, double zEnd)
   {
      SimpleTableTerrainObject table = new SimpleTableTerrainObject(xStart, yStart, xEnd, yEnd, zStart, zEnd);
      addTerrainObject(table);
   }

   public void addRotatableTable(RigidBodyTransform configuration, double xLength, double yWidth, double zLength, double tableTopThickness)
   {
      RotatableTableTerrainObject box = new RotatableTableTerrainObject(configuration, xLength, yWidth, zLength, tableTopThickness);
      addTerrainObject(box);
   }

   public void addTerrainObject(TerrainObject3D object)
   {
      terrainObjects.add(object);
      linkGraphics.combine(object.getLinkGraphics());

      if (boundingBox == null)
      {
         boundingBox = new BoundingBox3D(object.getBoundingBox());
      }
      else
      {
         boundingBox = BoundingBox3D.union(boundingBox, object.getBoundingBox());
      }
   }

   public void addStaticLinkGraphics(Graphics3DObject linkGraphics)
   {
      this.linkGraphics.combine(linkGraphics);
   }

   public ArrayList<TerrainObject3D> getTerrainObjects()
   {
      return terrainObjects;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return this.linkGraphics;
   }

   private final Point3D localIntersection = new Point3D();
   private final Vector3D localNormal = new Vector3D();

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      double smallestDistance = Double.MAX_VALUE;
      boolean isInside = false;

      tempPointToCheck.set(x, y, z);

      // Pre-set some values, in case no object is close.
      intersectionToPack.set(x, y, 0.0);
      normalToPack.set(0.0, 0.0, 1.0);

      for (int i = 0; i < terrainObjects.size(); i++)
      {
         TerrainObject3D terrainObject = terrainObjects.get(i);
         if (terrainObject.isClose(x, y, z))
         {
            boolean localIsInside = terrainObject.checkIfInside(x, y, z, localIntersection, localNormal);

            if (localIsInside && (tempPointToCheck.distance(localIntersection) < smallestDistance))
            {
               smallestDistance = tempPointToCheck.distance(localIntersection);
               intersectionToPack.set(localIntersection);
               normalToPack.set(localNormal);
               isInside = true;
            }
         }
      }

      // Reset pointToCheck for rewindability tests
      tempPointToCheck.set(0.0, 0.0, 0.0);

      return isInside;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if (boundingBox == null)
         return false;

      return boundingBox.isInsideInclusive(x, y, z);
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   public void recursivelyAddBoundingBoxVisualizerToLinkGraphics(AppearanceDefinition appearance)
   {
      recursivelyAddBoundingBoxVisualizerToLinkGraphics(linkGraphics, appearance);
   }

   public void recursivelyAddBoundingBoxVisualizerToLinkGraphics(Graphics3DObject linkGraphics, AppearanceDefinition appearance)
   {
      addBoundingBoxVisualizerToLinkGraphics(linkGraphics, appearance);

      for (TerrainObject3D terrainObject : terrainObjects)
      {
         if (terrainObject instanceof CombinedTerrainObject3D)
         {
            ((CombinedTerrainObject3D) terrainObject).recursivelyAddBoundingBoxVisualizerToLinkGraphics(linkGraphics, appearance);
         }
         else
         {
         }
      }
   }

   public void addBoundingBoxVisualizerToLinkGraphics(Graphics3DObject linkGraphics, AppearanceDefinition appearance)
   {
      linkGraphics.identity();
      double centerX = (boundingBox.getMinX() + boundingBox.getMaxX()) / 2.0;
      double centerY = (boundingBox.getMinY() + boundingBox.getMaxY()) / 2.0;
      double centerZ = boundingBox.getMinZ();

      linkGraphics.translate(centerX, centerY, centerZ);

      double xLength = boundingBox.getMaxX() - boundingBox.getMinX();
      double yLength = boundingBox.getMaxY() - boundingBox.getMinY();
      double zLength = boundingBox.getMaxZ() - boundingBox.getMinZ();

      linkGraphics.addCube(xLength, yLength, zLength, appearance);
   }

   public void recursivelyPrintBoundingBoxes(StringBuffer stringBuffer)
   {
      stringBuffer.append("\n");
      stringBuffer.append(name + "\n");
      stringBuffer.append(boundingBox);
      stringBuffer.append("\n");

      for (TerrainObject3D terrainObject : terrainObjects)
      {
         if (terrainObject instanceof CombinedTerrainObject3D)
         {
            ((CombinedTerrainObject3D) terrainObject).recursivelyPrintBoundingBoxes(stringBuffer);
         }
         else
         {
            stringBuffer.append(terrainObject.getBoundingBox());
            stringBuffer.append("\n");
         }
      }
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      double heightAt = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < terrainObjects.size(); i++)
      {
         TerrainObject3D terrainObject = terrainObjects.get(i);

         if (terrainObject.isClose(x, y, z))
         {
            HeightMap heightMap = terrainObject.getHeightMapIfAvailable();
            if (heightMap != null)
            {
               double localHeightAt = heightMap.heightAt(x, y, z);
               if (localHeightAt > heightAt)
               {
                  heightAt = localHeightAt;
               }
            }
         }
      }

      return heightAt;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < terrainObjects.size(); i++)
      {
         TerrainObject3D terrainObject = terrainObjects.get(i);

         if (terrainObject.isClose(x, y, z))
         {
            HeightMapWithNormals heightMap = terrainObject.getHeightMapIfAvailable();
            if (heightMap != null)
            {
               double localHeightAt = heightMap.heightAt(x, y, z);
               if (localHeightAt > heightAt)
               {
                  heightAt = heightMap.heightAndNormalAt(x, y, z, normalToPack);
               }
            }
         }
      }

      return heightAt;
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

}
