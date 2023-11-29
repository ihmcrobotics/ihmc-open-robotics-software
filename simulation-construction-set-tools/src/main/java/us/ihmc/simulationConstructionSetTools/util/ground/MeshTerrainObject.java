package us.ihmc.simulationConstructionSetTools.util.ground;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.io.FilenameUtils;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.MTLLoader;
import com.jme3.scene.plugins.OBJLoader;
import com.jme3.texture.plugins.AWTLoader;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.physics.CollidableVisualizer;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.io.JSONFileTools;

/**
 * MeshTerrain Object creates a terrain Object that decomposes a Concave mesh into convex parts
 * using Khaled Mamou's VHACD algorithm The convex parts are then combined together to make a
 * MeshTerrainObject.
 * 
 * @author Khizar Mohammed Amjed Mohamed
 */
public class MeshTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   public static final String VHACD_FILENAME_EXTENSION = "_VHACDParameters.json";

   private static final double EPSILON = 1.0e-12;
   private final BoundingBox3D boundingBox = new BoundingBox3D();
   private final List<ConvexPolytope3D> convexPolytopes = new ArrayList<>();
   private final Graphics3DObject linkGraphics;
   private final RigidBodyTransformReadOnly pose;

   private final MeshTerrainObjectParameters parameters;

   public MeshTerrainObject(String filePath)
   {
      this(useTunedMeshTerrainObjectParametersIfItExists(filePath));
   }

   public MeshTerrainObject(MeshTerrainObjectParameters vhacdParameters)
   {
      this(vhacdParameters, new RigidBodyTransform());
   }

   public MeshTerrainObject(String filePath, RigidBodyTransformReadOnly transform)
   {
      this(useTunedMeshTerrainObjectParametersIfItExists(filePath), transform);
   }

   public MeshTerrainObject(MeshTerrainObjectParameters vhacdParameters, RigidBodyTransformReadOnly transform)
   {
      String filePath = vhacdParameters.getModelDirectory();

      if (transform == null)
      {
         pose = new RigidBodyTransform();
      }
      else
      {
         pose = transform;
      }

      linkGraphics = new Graphics3DObject();
      this.parameters = vhacdParameters;

      doDecomposition(filePath);

      boundingBox.setToNaN();
      convexPolytopes.forEach(polytope -> boundingBox.combine(polytope.getBoundingBox()));

      if (this.getParameters().isShowUndecomposedMeshGraphics() && this.getParameters().isShowDecomposedMeshGraphics())
      {
         Random random = new Random(83432);
         for (ConvexPolytope3D convexPolytope : convexPolytopes)
         {
            linkGraphics.addMeshData(CollidableVisualizer.newConvexPolytope3DMesh(convexPolytope), YoAppearance.randomColor(random));
         }
         linkGraphics.transform(pose);
         linkGraphics.addModelFile(filePath);

      }

      else if (this.getParameters().isShowUndecomposedMeshGraphics())
      {
         linkGraphics.transform(pose);
         linkGraphics.addModelFile(filePath);

      }

      else if (this.getParameters().isShowDecomposedMeshGraphics())
      {
         Random random = new Random(83432);
         for (ConvexPolytope3D convexPolytope : convexPolytopes)
         {
            linkGraphics.addMeshData(CollidableVisualizer.newConvexPolytope3DMesh(convexPolytope), YoAppearance.randomColor(random));
         }
      }
   }

   public static MeshTerrainObjectParameters useTunedMeshTerrainObjectParametersIfItExists(String filePath)
   {
      MeshTerrainObjectParameters parameters = new MeshTerrainObjectParameters(filePath);
      InputStream inputStream = null;
      String jsonFilePath = FilenameUtils.removeExtension(filePath) + VHACD_FILENAME_EXTENSION;

      File vhacdParametersJsonFile = new File(jsonFilePath);

      if (vhacdParametersJsonFile.exists())
      {
         try
         {
            inputStream = new FileInputStream(vhacdParametersJsonFile);
         }
         catch (FileNotFoundException e)
         {
            e.printStackTrace();
         }
      }
      else
      {
         inputStream = parameters.getClass().getClassLoader().getResourceAsStream(jsonFilePath);
      }

      if (inputStream == null)
      {
         // If the inputStream is null it's likely because the file doesn't exist or got moved. Check file path
         LogTools.info(jsonFilePath + " was not found. Using defualt parameters istead");
      }
      else
      {
         JSONFileTools.load(inputStream, rootNode ->
         {
            parameters.setShowDecomposedMeshGraphics(rootNode.get("showDecomposedMeshGraphics").asBoolean());
            parameters.setShowUndecomposedMeshGraphics(rootNode.get("showRawMeshGraphics").asBoolean());
            parameters.setDoConvexDecomposition(rootNode.get("doConvexDecomposition").asBoolean());

            parameters.setMaxNoOfHulls(rootNode.get("maximumNumberOfHulls").asInt());
            parameters.setMaxNoOfVertices(rootNode.get("maximumNumberOfVerticesPerHull").asInt());
            parameters.setVoxelResolution(rootNode.get("maximumVoxelResolution").asInt());
            parameters.setMaxVolumePercentError(rootNode.get("maximumVolumetricPercentError").asDouble());
         });
      }
      return parameters;
   }

   private void doDecomposition(String filePath)
   {
      Spatial model = loadOBJFromPath(filePath);

      if (!getParameters().isDoConvexDecomposition())
      {
         makeConvexPolytopeFromHullCollissionShape(CollisionShapeFactory.createMergedHullShape(model));
      }
      else
      {

         CompoundCollisionShape compundShapes = CollisionShapeFactory.createVhacdShape(model, getParameters().getVhacd4Parameters(), null);

         for (ChildCollisionShape childshape : compundShapes.listChildren())
         {
            makeConvexPolytopeFromHullCollissionShape((HullCollisionShape) childshape.getShape());
         }
      }
   }

   private void makeConvexPolytopeFromHullCollissionShape(HullCollisionShape hullShape)
   {
      float[] hullVertices = hullShape.copyHullVertices();
      ArrayList<Point3D> verticesList = new ArrayList<Point3D>();

      for (int vertexID = 0; vertexID < hullVertices.length; vertexID += 3)
      {
         Point3D vertex = new Point3D(hullVertices[vertexID], hullVertices[vertexID + 1], hullVertices[vertexID + 2]);
         vertex.applyTransform(pose);
         verticesList.add(vertex);
      }

      ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(verticesList));
      convexPolytopes.add(convexPolytope3D);
   }

   private Spatial loadOBJFromPath(String objAssetPath)
   {

      DesktopAssetManager assetManager = new DesktopAssetManager();
      assetManager.registerLoader(AWTLoader.class, "jpg", "png");
      assetManager.registerLoader(OBJLoader.class, "obj");
      assetManager.registerLoader(MTLLoader.class, "mtl");
      assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
      assetManager.registerLocator(null, ClasspathLocator.class);
      assetManager.registerLocator(null, FileLocator.class);

      Spatial model = assetManager.loadModel(objAssetPath);
      return model;
   }

   public IntersectionResult intersectionWithVerticalLine(double x, double y)
   {
      Point3D point = new Point3D(x, y, 0);

      double highest = Double.NEGATIVE_INFINITY;
      double lowest = Double.POSITIVE_INFINITY;

      Face3DReadOnly highestFace = null;
      Face3DReadOnly lowestFace = null;

      for (ConvexPolytope3D convexPolytope : convexPolytopes)
      {
         // Find the highest and lowest points on the faces of the ConvexPolytope that
         // the point passes through along the Z-Axis.
         // If may go through multiple points or no point.
         for (Face3DReadOnly face : convexPolytope.getFaces())
         {
            Point3D pointIntersectionLineAndFace = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(face.getCentroid(),
                                                                                                           face.getNormal(),
                                                                                                           point,
                                                                                                           Axis3D.Z);
            if (pointIntersectionLineAndFace != null)
            {
               if (face.distance(pointIntersectionLineAndFace) <= EPSILON)
               {
                  if (pointIntersectionLineAndFace.getZ() > highest)
                  {
                     highest = pointIntersectionLineAndFace.getZ();
                     highestFace = face;
                  }
                  if (pointIntersectionLineAndFace.getZ() < lowest)
                  {
                     lowest = pointIntersectionLineAndFace.getZ();
                     lowestFace = face;
                  }
               }
            }
         }
      }

      return new IntersectionResult(new Point3D(x, y, highest), highestFace, new Point3D(x, y, lowest), lowestFace);

   }

   static class IntersectionResult
   {
      private final Point3D highestIntersection;
      private final Face3DReadOnly highestFace;

      private final Point3D lowestIntersection;
      private final Face3DReadOnly lowestFace;

      public IntersectionResult(Point3D highestIntersection, Face3DReadOnly highestFace, Point3D lowestIntersection, Face3DReadOnly lowestFace)
      {
         this.highestIntersection = highestIntersection;
         this.highestFace = highestFace;
         this.lowestIntersection = lowestIntersection;
         this.lowestFace = lowestFace;
      }

      public boolean isHighestPointValid()
      {
         return highestIntersection.getZ() != Double.NEGATIVE_INFINITY;
      }

      public Point3D getHighestIntersection()
      {
         return highestIntersection;
      }

      public Face3DReadOnly getHighestFace()
      {
         return highestFace;
      }

      public Point3D getLowestIntersection()
      {
         return lowestIntersection;
      }

      public Face3DReadOnly getLowestFace()
      {
         return lowestFace;
      }
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      IntersectionResult result = intersectionWithVerticalLine(x, y);
      // Determine the height of the point on the zAxis of the ConvexPolytope -
      // returns boundingBox.getMinZ() if it does
      // not pass through the ConvexPolytope or the point is completely below the
      // ConvexPolytope
      if (result.isHighestPointValid() && z >= result.lowestIntersection.getZ())
         return result.highestIntersection.getZ();

      return Double.NEGATIVE_INFINITY;
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      IntersectionResult result = intersectionWithVerticalLine(x, y);

      double heightAt;

      if (result.isHighestPointValid() && z >= result.lowestIntersection.getZ())
         heightAt = result.highestIntersection.getZ();
      else
         heightAt = Double.NEGATIVE_INFINITY;

      if (intersectionToPack != null)
      {
         intersectionToPack.setX(x);
         intersectionToPack.setY(y);
         intersectionToPack.setZ(heightAt);
      }

      if (normalToPack != null && heightAt > Double.NEGATIVE_INFINITY)
      {
         normalToPack.set(result.highestFace.getNormal());
      }

      return (z < heightAt);
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      IntersectionResult result = intersectionWithVerticalLine(x, y);

      double heightAt;

      if (result.isHighestPointValid() && z >= result.lowestIntersection.getZ())
         heightAt = result.highestIntersection.getZ();
      else
         heightAt = Double.NEGATIVE_INFINITY;
      if (normalToPack != null && heightAt > Double.NEGATIVE_INFINITY)
      {
         normalToPack.set(result.highestFace.getNormal());
      }

      return heightAt;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if (boundingBox == null)
         return false;

      return boundingBox.isInsideInclusive(x, y, z);
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

   @Override
   public List<? extends Shape3DReadOnly> getTerrainCollisionShapes()
   {
      return convexPolytopes;
   }

   public MeshTerrainObjectParameters getParameters()
   {
      return parameters;
   }

}