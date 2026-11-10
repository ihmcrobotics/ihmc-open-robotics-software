package us.ihmc.rdx.simulation.environment.isaac;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.simulation.environment.mujoco.RDXMujocoCollisionShape;
import us.ihmc.rdx.simulation.environment.mujoco.RDXMujocoObjectCollisionSync;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXDirectionalLightObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXPointLightObject;
import us.ihmc.rdx.simulation.environment.object.objects.terrain.RDXProceduralGroundObject;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainBox;
import us.ihmc.simulationConstructionSetTools.util.environments.procedural.ProceduralTerrainGeometry.TerrainSurface;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Writes an RDX environment as a USD stage Isaac Sim opens directly: the visuals RDX draws, the
 * collisions MuJoCo simulates, free bodies for the interactable objects, articulations for the
 * jointed objects, the lights, a ground plane and a physics scene. The robot is not included.
 * <p>
 * Must run on the libGDX thread: meshes and textures are read from the loaded models.
 */
public class RDXIsaacSimEnvironmentExporter
{
   /** Isaac Sim lights are in nits. These roughly match the brightness of an RDX scene. */
   private static final double POINT_LIGHT_INTENSITY = 30000.0;
   private static final double POINT_LIGHT_RADIUS = 0.5;
   private static final double DIRECTIONAL_LIGHT_INTENSITY = 1000.0;
   private static final double DOME_LIGHT_INTENSITY = 1000.0;

   private final Set<String> warnedMissingHulls = new HashSet<>();
   private final Set<String> warnedUnsupportedTypes = new HashSet<>();
   private final Set<String> usedNames = new HashSet<>();
   private final Pool<Renderable> renderablePool = new Pool<>()
   {
      @Override
      protected Renderable newObject()
      {
         return new Renderable();
      }
   };
   private final Matrix4 identity = new Matrix4();
   private RDXUSDAWriter usd;
   private int objectCount;
   /** Keeps mesh prim names unique when several models are written under one prim. */
   private int modelCount;
   private int articulationCount;
   private int lightCount;

   /**
    * @param environmentName the stage is {@code <directory>/<environmentName>.usda}
    * @param ambientLight    RDX ambient light amount, 0 to 1, which scales the dome light
    * @return the written stage
    */
   public Path export(String environmentName, List<RDXEnvironmentObject> objects, double ambientLight, Path directory) throws IOException
   {
      Files.createDirectories(directory);
      usd = new RDXUSDAWriter(directory);
      usedNames.clear();
      objectCount = 0;
      articulationCount = 0;
      lightCount = 0;

      usd.header("RDX environment " + environmentName + ", exported for Isaac Sim");
      usd.beginPrim("Xform", "World");
      writePhysicsScene();
      if (objects.stream().noneMatch(RDXProceduralGroundObject.class::isInstance))
         writeGroundPlane();
      writeDomeLight(ambientLight);

      for (RDXEnvironmentObject object : objects)
      {
         try
         {
            if (object instanceof RDXPointLightObject || object instanceof RDXDirectionalLightObject)
               writeLight(object);
            else if (object instanceof RDXIsaacSimArticulatedObject articulated)
               writeArticulation(object, articulated);
            else
               writeObject(object);
         }
         catch (RuntimeException exception)
         {
            LogTools.error("Skipped {} in the Isaac Sim export: {}", object.getTitleCasedName(), exception.getMessage());
         }
      }

      usd.looks();
      usd.endPrim();
      Path stage = directory.resolve(RDXUSDAWriter.primName(environmentName) + ".usda");
      usd.save(stage);
      LogTools.info("Exported " + objectCount + " object(s), " + articulationCount + " articulation(s), " + lightCount + " light(s), "
                    + usd.getMeshCount() + " mesh(es) and " + usd.getTextureCount() + " texture(s) to " + stage);
      return stage;
   }

   private void writePhysicsScene()
   {
      usd.beginPrim("PhysicsScene", "PhysicsScene");
      usd.attribute("vector3f physics:gravityDirection", "(0, 0, -1)");
      usd.attribute("float physics:gravityMagnitude", "9.81");
      usd.endPrim();
   }

   /** The MuJoCo world has a flat ground at z = 0. Invisible, so it does not cover the lab floor. */
   private void writeGroundPlane()
   {
      usd.beginPrim("Plane", "GroundPlane", "PhysicsCollisionAPI");
      usd.attribute("uniform token axis", "\"Z\"");
      usd.attribute("double length", "200");
      usd.attribute("double width", "200");
      usd.attribute("token visibility", "\"invisible\"");
      usd.endPrim();
   }

   private void writeDomeLight(double ambientLight)
   {
      usd.beginPrim("DomeLight", "AmbientLight");
      usd.attribute("float inputs:intensity", RDXUSDAWriter.fmt(DOME_LIGHT_INTENSITY * Math.max(0.2, Math.min(1.0, ambientLight * 2.0))));
      usd.endPrim();
   }

   private void writeLight(RDXEnvironmentObject object)
   {
      String name = uniqueName(object);
      if (object instanceof RDXPointLightObject)
      {
         usd.beginPrim("SphereLight", name);
         usd.attribute("float inputs:intensity", RDXUSDAWriter.fmt(POINT_LIGHT_INTENSITY));
         usd.attribute("float inputs:radius", RDXUSDAWriter.fmt(POINT_LIGHT_RADIUS));
         usd.attribute("bool treatAsPoint", "false");
      }
      else
      {
         // A distant light shines along its -Z
         usd.beginPrim("DistantLight", name);
         usd.attribute("float inputs:intensity", RDXUSDAWriter.fmt(DIRECTIONAL_LIGHT_INTENSITY));
         usd.attribute("float inputs:angle", "0.53");
      }
      usd.transform(object.getObjectTransform());
      usd.endPrim();
      lightCount++;
   }

   /** One prim at the object pose: a rigid body when MuJoCo makes it a free body, static otherwise. */
   private void writeObject(RDXEnvironmentObject object)
   {
      Array<Renderable> renderables = new Array<>();
      object.getRealRenderables(renderables, renderablePool);
      RDXMujocoCollisionShape shape = RDXMujocoObjectCollisionSync.toCollisionShape(object, warnedMissingHulls, warnedUnsupportedTypes);
      if (renderables.isEmpty() && shape == null)
      {
         renderablePool.freeAll(renderables);
         return;
      }

      boolean dynamic = shape != null && shape.isDynamic();
      RigidBodyTransform objectToWorld = new RigidBodyTransform(object.getObjectTransform());
      String name = uniqueName(object);
      if (dynamic)
         usd.beginPrim("Xform", name, "PhysicsRigidBodyAPI", "PhysicsMassAPI");
      else
         usd.beginPrim("Xform", name);
      if (dynamic && object.getMass() > 0.0f)
         usd.attribute("float physics:mass", RDXUSDAWriter.fmt(object.getMass()));
      usd.transform(objectToWorld);

      Matrix4 objectToWorldMatrix = new Matrix4();
      LibGDXTools.toLibGDX(objectToWorld, objectToWorldMatrix);
      if (!renderables.isEmpty())
      {
         usd.beginPrim("Xform", "Visuals");
         usd.meshes(renderables, objectToWorldMatrix, "Mesh", false);
         usd.endPrim();
      }
      renderablePool.freeAll(renderables);

      if (shape != null)
      {
         RigidBodyTransform collisionToObject = new RigidBodyTransform(objectToWorld);
         collisionToObject.invert();
         collisionToObject.multiply(new RigidBodyTransform(shape.getPoseInWorld().getOrientation(), shape.getPoseInWorld().getPosition()));
         usd.beginPrim("Xform", "Collisions");
         writeCollision(shape, collisionToObject);
         usd.endPrim();
      }
      usd.endPrim();
      objectCount++;
   }

   private void writeCollision(RDXMujocoCollisionShape shape, RigidBodyTransform collisionToObject)
   {
      switch (shape.getType())
      {
         case BOX -> writeBox("Box", shape.getSizeX(), shape.getSizeY(), shape.getSizeZ(), collisionToObject);
         case SPHERE ->
         {
            usd.beginPrim("Sphere", "Sphere", "PhysicsCollisionAPI");
            usd.attribute("double radius", RDXUSDAWriter.fmt(shape.getSizeX()));
            usd.attribute("uniform token purpose", "\"guide\"");
            usd.transform(collisionToObject);
            usd.endPrim();
         }
         case MESH -> writeCollisionModel(RDXModelLoader.load(shape.getMeshResourcePath()), collisionToObject, "Hull");
         case TERRAIN ->
         {
            int index = 0;
            for (TerrainBox box : shape.getTerrainBoxes())
            {
               RigidBodyTransform boxToObject = new RigidBodyTransform(collisionToObject);
               boxToObject.appendTranslation(box.x(), box.y(), box.z());
               writeBox("Box_" + index++, box.sizeX(), box.sizeY(), box.sizeZ(), boxToObject);
            }
            index = 0;
            for (TerrainSurface surface : shape.getTerrainSurfaces())
               writePrism("Surface_" + index++, surface, collisionToObject);
         }
      }
   }

   private void writeBox(String name, double sizeX, double sizeY, double sizeZ, RigidBodyTransformReadOnly boxToParent)
   {
      usd.beginPrim("Cube", name, "PhysicsCollisionAPI");
      usd.attribute("double size", "1");
      usd.attribute("uniform token purpose", "\"guide\"");
      usd.transformWithScale(boxToParent, sizeX, sizeY, sizeZ);
      usd.endPrim();
   }

   /** A terrain surface filled down to its bottom, as MuJoCo collides with it. */
   private void writePrism(String name, TerrainSurface surface, RigidBodyTransformReadOnly surfaceToParent)
   {
      List<Point3D> top = surface.vertices();
      int n = top.size();
      if (n < 3)
         return;
      usd.beginPrim("Mesh", name, "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI");
      StringBuilder points = new StringBuilder("[");
      for (int i = 0; i < 2 * n; i++)
      {
         Point3D vertex = top.get(i % n);
         double z = i < n ? vertex.getZ() : surface.bottomZ();
         points.append(i == 0 ? "(" : ", (").append(RDXUSDAWriter.fmt(vertex.getX())).append(", ")
               .append(RDXUSDAWriter.fmt(vertex.getY())).append(", ").append(RDXUSDAWriter.fmt(z)).append(')');
      }
      StringBuilder counts = new StringBuilder("[").append(n).append(", ").append(n);
      StringBuilder indices = new StringBuilder("[");
      for (int i = 0; i < n; i++)
         indices.append(i == 0 ? "" : ", ").append(i);
      for (int i = n - 1; i >= 0; i--)
         indices.append(", ").append(n + i);
      for (int i = 0; i < n; i++)
      {
         int next = (i + 1) % n;
         counts.append(", 4");
         indices.append(", ").append(i).append(", ").append(n + i).append(", ").append(n + next).append(", ").append(next);
      }
      usd.attribute("int[] faceVertexCounts", counts.append(']').toString());
      usd.attribute("int[] faceVertexIndices", indices.append(']').toString());
      usd.attribute("point3f[] points", points.append(']').toString());
      usd.attribute("uniform token physics:approximation", "\"convexHull\"");
      usd.attribute("uniform token purpose", "\"guide\"");
      usd.transform(surfaceToParent);
      usd.endPrim();
   }

   private void writeCollisionModel(Model model, RigidBodyTransformReadOnly modelToParent, String prefix)
   {
      writeModel(model, modelToParent, prefix, true);
   }

   private void writeModel(Model model, RigidBodyTransformReadOnly modelToParent, String prefix, boolean collision)
   {
      ModelInstance instance = new ModelInstance(model);
      LibGDXTools.toLibGDX(new RigidBodyTransform(modelToParent), instance.transform);
      Array<Renderable> renderables = new Array<>();
      instance.getRenderables(renderables, renderablePool);
      usd.meshes(renderables, identity, prefix + "_" + modelCount++, collision);
      renderablePool.freeAll(renderables);
   }

   /**
    * A fixed-base articulation: each link a rigid body directly under the object prim, posed at the
    * current joint positions, and a joint per link. The root is welded to the world.
    */
   private void writeArticulation(RDXEnvironmentObject object, RDXIsaacSimArticulatedObject articulated)
   {
      List<RDXIsaacSimLink> links = articulated.getIsaacSimLinks();
      if (links.isEmpty())
         return;
      String name = uniqueName(object);
      String objectPath = "/World/" + name;
      RigidBodyTransform objectToWorld = new RigidBodyTransform(object.getObjectTransform());

      List<String> linkNames = new ArrayList<>();
      Map<String, Integer> linkIndex = new HashMap<>();
      Set<String> linkNameSet = new HashSet<>();
      for (int i = 0; i < links.size(); i++)
      {
         String linkName = RDXUSDAWriter.primName(links.get(i).getName());
         while (!linkNameSet.add(linkName))
            linkName = linkName + "_";
         linkNames.add(linkName);
         linkIndex.put(links.get(i).getName(), i);
      }
      // The joints and the excluded pairs reference links by path
      List<List<String>> filteredPairs = new ArrayList<>();
      for (int i = 0; i < links.size(); i++)
         filteredPairs.add(new ArrayList<>());
      for (String[] pair : articulated.getIsaacSimExcludedLinkPairs())
      {
         Integer first = linkIndex.get(pair[0]);
         Integer second = linkIndex.get(pair[1]);
         if (first != null && second != null)
            filteredPairs.get(first).add(objectPath + "/" + linkNames.get(second));
      }

      RigidBodyTransform[] linkToObject = new RigidBodyTransform[links.size()];
      AxisAngle axisAngle = new AxisAngle();
      for (int i = 0; i < links.size(); i++)
      {
         RDXIsaacSimLink link = links.get(i);
         linkToObject[i] = new RigidBodyTransform();
         if (link.getParentIndex() < 0)
            continue;
         linkToObject[i].set(linkToObject[link.getParentIndex()]);
         linkToObject[i].multiply(link.getJointFrameInParent());
         RigidBodyTransform motion = new RigidBodyTransform();
         switch (link.getJointType())
         {
            case REVOLUTE, CONTINUOUS ->
            {
               axisAngle.set(link.getAxis(), link.getPosition());
               motion.getRotation().set(axisAngle);
            }
            case PRISMATIC -> motion.getTranslation().setAndScale(link.getPosition(), link.getAxis());
            default ->
            {
            }
         }
         linkToObject[i].multiply(motion);
      }

      usd.beginPrim("Xform", name, "PhysicsArticulationRootAPI");
      usd.transform(objectToWorld);
      for (int i = 0; i < links.size(); i++)
      {
         RDXIsaacSimLink link = links.get(i);
         if (filteredPairs.get(i).isEmpty())
            usd.beginPrim("Xform", linkNames.get(i), "PhysicsRigidBodyAPI", "PhysicsMassAPI");
         else
            usd.beginPrim("Xform", linkNames.get(i), "PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsFilteredPairsAPI");
         if (link.getMass() > 0.0)
            usd.attribute("float physics:mass", RDXUSDAWriter.fmt(link.getMass()));
         else if (link.getDensity() > 0.0)
            usd.attribute("float physics:density", RDXUSDAWriter.fmt(link.getDensity()));
         if (link.getCenterOfMass() != null)
            usd.attribute("point3f physics:centerOfMass", RDXUSDAWriter.tuple(link.getCenterOfMass()));
         if (!filteredPairs.get(i).isEmpty())
            usd.relationship("physics:filteredPairs", filteredPairs.get(i).toArray(new String[0]));
         usd.transform(linkToObject[i]);
         if (!link.getVisuals().isEmpty())
         {
            usd.beginPrim("Xform", "Visuals");
            for (RDXIsaacSimLink.Visual visual : link.getVisuals())
               writeModel(visual.model(), visual.poseInLink(), "Mesh", false);
            usd.endPrim();
         }
         if (!link.getCollisions().isEmpty())
         {
            usd.beginPrim("Xform", "Collisions");
            int index = 0;
            for (RDXIsaacSimLink.Collision collision : link.getCollisions())
            {
               if (collision.model() != null)
                  writeCollisionModel(collision.model(), collision.poseInLink(), "Hull");
               else
                  writeBox("Box_" + index++, collision.size().getX(), collision.size().getY(), collision.size().getZ(), collision.poseInLink());
            }
            usd.endPrim();
         }
         usd.endPrim();
      }

      usd.beginPrim("Scope", "Joints");
      for (int i = 0; i < links.size(); i++)
         writeJoint(objectPath, linkNames, links, i, objectToWorld);
      usd.endPrim();
      usd.endPrim();
      articulationCount++;
   }

   /**
    * USD joints turn about or slide along the X, Y or Z axis of their frame, so the joint frame on
    * each side is the URDF joint frame rotated to put the URDF axis on X.
    */
   private void writeJoint(String objectPath, List<String> linkNames, List<RDXIsaacSimLink> links, int index, RigidBodyTransformReadOnly objectToWorld)
   {
      RDXIsaacSimLink link = links.get(index);
      String childPath = objectPath + "/" + linkNames.get(index);
      if (link.getJointType() == RDXIsaacSimLink.JointType.ROOT)
      {
         // No body0: the first frame is in the world
         usd.beginPrim("PhysicsFixedJoint", linkNames.get(index) + "_to_world");
         usd.relationship("physics:body1", childPath);
         usd.attribute("point3f physics:localPos0", RDXUSDAWriter.tuple(objectToWorld.getTranslation()));
         usd.attribute("quatf physics:localRot0", RDXUSDAWriter.quaternion(new Quaternion(objectToWorld.getRotation())));
         usd.attribute("point3f physics:localPos1", "(0, 0, 0)");
         usd.attribute("quatf physics:localRot1", "(1, 0, 0, 0)");
         usd.endPrim();
         return;
      }

      String parentPath = objectPath + "/" + linkNames.get(link.getParentIndex());
      String jointName = link.getJointName() == null ? linkNames.get(index) + "_joint" : RDXUSDAWriter.primName(link.getJointName());
      RigidBodyTransformReadOnly jointFrame = link.getJointFrameInParent();
      if (link.getJointType() == RDXIsaacSimLink.JointType.FIXED)
      {
         usd.beginPrim("PhysicsFixedJoint", jointName);
         usd.relationship("physics:body0", parentPath);
         usd.relationship("physics:body1", childPath);
         usd.attribute("point3f physics:localPos0", RDXUSDAWriter.tuple(jointFrame.getTranslation()));
         usd.attribute("quatf physics:localRot0", RDXUSDAWriter.quaternion(new Quaternion(jointFrame.getRotation())));
         usd.attribute("point3f physics:localPos1", "(0, 0, 0)");
         usd.attribute("quatf physics:localRot1", "(1, 0, 0, 0)");
         usd.endPrim();
         return;
      }

      boolean prismatic = link.getJointType() == RDXIsaacSimLink.JointType.PRISMATIC;
      String kind = prismatic ? "linear" : "angular";
      // Isaac Sim takes angles, angular drive gains and angular limits in degrees
      double toUsd = prismatic ? 1.0 : 180.0 / Math.PI;
      Quaternion axisToX = new Quaternion();
      EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(new Vector3D(1.0, 0.0, 0.0), link.getAxis(), axisToX);
      Quaternion localRot0 = new Quaternion(jointFrame.getRotation());
      localRot0.multiply(axisToX);

      List<String> schemas = new ArrayList<>();
      schemas.add("PhysicsJointStateAPI:" + kind);
      boolean drive = link.getDamping() > 0.0 || link.getStiffness() > 0.0;
      if (drive)
         schemas.add("PhysicsDriveAPI:" + kind);
      usd.beginPrim(prismatic ? "PhysicsPrismaticJoint" : "PhysicsRevoluteJoint", jointName, schemas.toArray(new String[0]));
      usd.attribute("uniform token physics:axis", "\"X\"");
      usd.relationship("physics:body0", parentPath);
      usd.relationship("physics:body1", childPath);
      usd.attribute("point3f physics:localPos0", RDXUSDAWriter.tuple(jointFrame.getTranslation()));
      usd.attribute("quatf physics:localRot0", RDXUSDAWriter.quaternion(localRot0));
      usd.attribute("point3f physics:localPos1", "(0, 0, 0)");
      usd.attribute("quatf physics:localRot1", RDXUSDAWriter.quaternion(axisToX));
      if (link.getJointType() != RDXIsaacSimLink.JointType.CONTINUOUS && Double.isFinite(link.getLowerLimit()) && Double.isFinite(link.getUpperLimit()))
      {
         usd.attribute("float physics:lowerLimit", RDXUSDAWriter.fmt(link.getLowerLimit() * toUsd));
         usd.attribute("float physics:upperLimit", RDXUSDAWriter.fmt(link.getUpperLimit() * toUsd));
      }
      usd.attribute("float state:" + kind + ":physics:position", RDXUSDAWriter.fmt(link.getPosition() * toUsd));
      usd.attribute("float state:" + kind + ":physics:velocity", "0");
      if (drive)
      {
         // Per degree, so the gains scale the other way from the angles
         usd.attribute("uniform token drive:" + kind + ":physics:type", "\"force\"");
         usd.attribute("float drive:" + kind + ":physics:damping", RDXUSDAWriter.fmt(link.getDamping() / toUsd));
         usd.attribute("float drive:" + kind + ":physics:stiffness", RDXUSDAWriter.fmt(link.getStiffness() / toUsd));
         usd.attribute("float drive:" + kind + ":physics:targetPosition", "0");
      }
      usd.endPrim();
   }

   private String uniqueName(RDXEnvironmentObject object)
   {
      String name = RDXUSDAWriter.primName(object.getPascalCasedName() + "_" + object.getObjectIndex());
      while (!usedNames.add(name))
         name = name + "_";
      return name;
   }
}
