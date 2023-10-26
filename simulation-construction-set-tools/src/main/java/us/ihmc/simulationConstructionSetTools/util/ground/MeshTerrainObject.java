package us.ihmc.simulationConstructionSetTools.util.ground;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.asset.plugins.ClasspathLocator;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.NativePhysicsObject;
import com.jme3.bullet.collision.shapes.CompoundCollisionShape;
import com.jme3.bullet.collision.shapes.HullCollisionShape;
import com.jme3.bullet.collision.shapes.infos.ChildCollisionShape;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.material.plugins.J3MLoader;
import com.jme3.scene.Spatial;
import com.jme3.scene.plugins.MTLLoader;
import com.jme3.scene.plugins.OBJLoader;
import com.jme3.system.NativeLibraryLoader;
import com.jme3.texture.plugins.AWTLoader;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.Face3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.physics.CollidableVisualizer;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import vhacd.VHACDParameters;
import vhacd4.Vhacd4Parameters;

/**
 * MeshTerrain Object creates a terrain Object that decomposes a concave mesh into convex parts
 * using Khaled Mamou's VHACD algorithm The covex parts are then combined together to make a
 * MeshTerrainObject.
 * 
 * @author Khizar Mohammed Amjed Mohamed
 */
public class MeshTerrainObject implements TerrainObject3D, HeightMapWithNormals
{

   public enum ConvexDecomposition
   {
      VHACD, VHACD4, NO_DECOMPOSITION
   };
   
   public class DecompositionParameters
   {
      public double maxNoOfHulls;
      public double MaxNoOfVerticesPerHull;
      public double maxAllowableError;
      
      DecompositionParameters(double a, double b)
      {
         this.maxNoOfHulls=a;
      }
      public double geta(){
         return maxNoOfHulls;
      }
   }

   private final BoundingBox3D boundingBox = new BoundingBox3D();
   private final List<ConvexPolytope3D> convexPolytopes = new ArrayList<>();
   private final Graphics3DObject linkGraphics;
   private final ConvexDecomposition decompositionType;

   private boolean useOriginalMeshGraphics = false;
   private AppearanceDefinition meshAppearance = YoAppearance.Transparent();

   private int maxHulls = 20;
   private int maxVerticesPerHull = 64;
   private double maxError = 0.01;
   private int VoxelResolution = 100000;

   private double maxConcavity = 0.5;

   private static final double EPSILON = 1.0e-12;

   public MeshTerrainObject(String filename)
   {
      this(filename, ConvexDecomposition.VHACD4);
   }

   public MeshTerrainObject(String filename, ConvexDecomposition convexDecomposition)
   {
      this(filename, convexDecomposition, new RigidBodyTransform());
   }

   public MeshTerrainObject(String filename, RigidBodyTransformReadOnly transform)
   {
      this(filename, ConvexDecomposition.VHACD4, transform);
   }
   public MeshTerrainObject(String filename, ConvexDecomposition convexDecomposition, DecompositionParameters params)
   {
      this(filename, convexDecomposition, new RigidBodyTransform());
   }

   public MeshTerrainObject(String filename, ConvexDecomposition convexDecomposition, RigidBodyTransformReadOnly transform)
   {
      // Example for loading file from the resources folder
      // getClass().getClassLoader().getResource("fiducials/png/fiducial100.png");

      // String sampleFileName = "models/IronMan/IronMan.obj";
      // String sampleFileName = "models/Doorway/doorway.obj";

      RigidBodyTransformReadOnly pose;
      if (transform == null)
      {
         pose = new RigidBodyTransform();
      }
      else
      {
         pose = transform;
      }

      linkGraphics = new Graphics3DObject();
      decompositionType = convexDecomposition;

      doDecomposition(filename);

      convexPolytopes.forEach(polytope -> polytope.applyTransform(pose));
      boundingBox.setToNaN();
      convexPolytopes.forEach(polytope -> boundingBox.combine(polytope.getBoundingBox()));

      if (useOriginalMeshGraphics)
      {
         linkGraphics.transform(pose);
         linkGraphics.addModelFile(filename);
      }

      else
      {
         for (ConvexPolytope3D convexPolytope : convexPolytopes)
         {
            linkGraphics.addMeshData(CollidableVisualizer.newConvexPolytope3DMesh(convexPolytope), (AppearanceDefinition) meshAppearance);
         }
      }
   }

   private Collection<? extends ConvexPolytope3D> doDecomposition(String filename)
   {
      // filename.toExternalForm(); // That should be the absolute path to file
      NativeLibraryLoader.loadNativeLibrary("bulletjme", true);
      Spatial model = loadOBJFromPath(filename);

      CompoundCollisionShape compundShapes = decompose(model);

      addConvexPolytopesToList(compundShapes);

      return null;
   }

   private void addConvexPolytopesToList(CompoundCollisionShape compundShapes)
   {
      for (ChildCollisionShape childshape : compundShapes.listChildren())
      {
         HullCollisionShape hullShape = (HullCollisionShape) childshape.getShape();
         float[] hullVertices = hullShape.copyHullVertices();
         ArrayList<Point3D> verticesList = new ArrayList<Point3D>();

         for (int vertexID = 0; vertexID < hullVertices.length; vertexID += 3)
         {
            Point3D vertex = new Point3D(hullVertices[vertexID], hullVertices[vertexID + 1], hullVertices[vertexID + 2]);
            verticesList.add(vertex);
         }

         ConvexPolytope3D convexPolytope3D = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(verticesList));
         convexPolytopes.add(convexPolytope3D);
      }
   }

   private CompoundCollisionShape decompose(Spatial model)
   {
      // TODO Auto-generated method stub
      switch (decompositionType)
      {
         case VHACD:
            return CollisionShapeFactory.createVhacdShape(model, getVHACDParameters(), null);

         case VHACD4:
            return CollisionShapeFactory.createVhacdShape(model, getVhacd4Parameters(), null);
         case NO_DECOMPOSITION:
            break;
      }
      return null;

   }

   private Spatial loadOBJFromPath(String objAssetPath)
   {
      // TODO Auto-generated method stub

      DesktopAssetManager assetManager = new DesktopAssetManager();
      // assetManager.registerLoader(, null);
      assetManager.registerLoader(AWTLoader.class, "jpg", "png");
      assetManager.registerLoader(OBJLoader.class, "obj");
      assetManager.registerLoader(MTLLoader.class, "mtl");
      assetManager.registerLoader(J3MLoader.class, "j3m", "j3md");
      assetManager.registerLocator(null, ClasspathLocator.class);
      assetManager.registerLocator(null, FileLocator.class);

      Spatial model = assetManager.loadModel(objAssetPath);
      return model;
   }

   private Vhacd4Parameters getVhacd4Parameters()
   {
      Vhacd4Parameters paramters = new Vhacd4Parameters();
      paramters.setMaxVerticesPerHull(maxVerticesPerHull);
      paramters.setVoxelResolution(VoxelResolution);
      paramters.setVolumePercentError(maxError);
      paramters.setMaxHulls(maxHulls);
      return paramters;
   }

   private VHACDParameters getVHACDParameters()
   {
      VHACDParameters paramters = new VHACDParameters();
      paramters.setMaxConcavity(maxConcavity);
      paramters.setMaxVerticesPerHull(maxVerticesPerHull);
      paramters.setVoxelResolution(VoxelResolution);
      return paramters;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      Point3D point = new Point3D(x, y, z);
      double highest = Double.NEGATIVE_INFINITY;
      double lowest = Double.POSITIVE_INFINITY;

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
                  highest = Math.max(highest, pointIntersectionLineAndFace.getZ());
                  lowest = Math.min(lowest, pointIntersectionLineAndFace.getZ());
               }
            }
         }
      }
      // Determine the height of the point on the zAxis of the ConvexPolytope -
      // returns boundingBox.getMinZ() if it does
      // not pass through the ConvexPolytope or the point is completely below the
      // ConvexPolytope
      if (highest != Double.NEGATIVE_INFINITY && z >= lowest)
         return highest;

      return Double.NEGATIVE_INFINITY;
   }

   private final Point3D tempIntersection = new Point3D();

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      double heightAt = heightAt(x, y, z);

      if (intersectionToPack != null)
      {
         intersectionToPack.setX(x);
         intersectionToPack.setY(y);
         intersectionToPack.setZ(heightAt);
      }

      if (normalToPack != null)
      {
         if (heightAt > Double.NEGATIVE_INFINITY)
         {
            tempIntersection.set(x, y, heightAt);

            List<Face3D> closesFacesList = new ArrayList<Face3D>();

            for (ConvexPolytope3D convexPolytope : convexPolytopes)
            {
               closesFacesList.addAll(convexPolytope.getFaces());
            }

            Face3D closestFace = iterateAndGetClosestFace(closesFacesList, tempIntersection);
            normalToPack.set(closestFace.getNormal());
         }
      }

      return (z < heightAt);
   }

   private Face3D iterateAndGetClosestFace(List<Face3D> closesFacesList, Point3D query)
   {
      if (closesFacesList.size() == 0)
         return null;
      if (closesFacesList.size() == 1)
         return closesFacesList.get(0);

      boolean isOutside = false;
      double maxNegativeDistance = Double.NEGATIVE_INFINITY;
      Face3DReadOnly closestFace = null;
      int faceIndex;

      // First assume the query is inside and use the less expensive
      for (faceIndex = 0; faceIndex < closesFacesList.size(); faceIndex++)
      {
         Face3DReadOnly face = closesFacesList.get(faceIndex);
         double signedDistanceToPlane = face.signedDistanceFromSupportPlane(query);
         if (signedDistanceToPlane < 0.0)
         {
            if (signedDistanceToPlane > maxNegativeDistance)
            {
               closestFace = face;
               maxNegativeDistance = signedDistanceToPlane;
            }
         }
         else
         {
            isOutside = true;
            break;
         }
      }

      if (isOutside)
      { // The query is outside.
         closestFace = closesFacesList.get(faceIndex);
         double closestFaceDistance = closestFace.distance(query);

         faceIndex++;

         for (; faceIndex < closesFacesList.size(); faceIndex++)
         {

            Face3DReadOnly face = closesFacesList.get(faceIndex);

            double signedDistanceToPlane = face.signedDistanceFromSupportPlane(query);
            if (!(signedDistanceToPlane >= 0.0))
               continue; // The query is below, cannot be the closest face.

            // Now use the more expensive Face3DReadOnly.distance(Point3DReadOnly)
            double candidateDistance = face.distance(query);

            if (candidateDistance < closestFaceDistance)
            {
               closestFace = face;
               closestFaceDistance = candidateDistance;
            }
         }
      }

      return (Face3D) closestFace;
   }

   public boolean isUseOriginalMeshGraphics()
   {
      return useOriginalMeshGraphics;
   }

   public void setUseOriginalMeshGraphics(boolean useOriginalMeshGraphics)
   {
      this.useOriginalMeshGraphics = useOriginalMeshGraphics;
   }

   public AppearanceDefinition getMeshAppearance()
   {
      return meshAppearance;
   }

   public void setMeshAppearance(AppearanceDefinition meshAppearance)
   {
      this.meshAppearance = meshAppearance;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      checkIfInside(x, y, z, null, normalToPack);

      return heightAt(x, y, z);
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

}