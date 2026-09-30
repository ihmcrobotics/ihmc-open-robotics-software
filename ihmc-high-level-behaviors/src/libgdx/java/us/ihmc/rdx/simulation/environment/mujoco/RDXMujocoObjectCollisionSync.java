package us.ihmc.rdx.simulation.environment.mujoco;

import org.bytedeco.javacpp.DoublePointer;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.environment.object.objects.RDXArticulatedDoorObject;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoMultiBodyDynamicsWorld;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.MujocoPhysicsEngine;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngine;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Copies RDX environment-builder collisions into the running MuJoCo model.
 * <p>
 * When a visual GLB has a precomputed {@code <stem>_convex.stl} beside it, that hull is the
 * collision mesh. Objects without one keep their box or sphere. An articulated door is not a
 * static geom: it is a dynamic mechanism (welded frame, hinged panel, sprung lever). Adding or
 * removing an object rebuilds the world XML and recompiles, keeping the robot state. Dragging a
 * static object only writes that geom's pose. Dragging a door writes the frame body pose.
 */
public class RDXMujocoObjectCollisionSync implements Controller
{
   private static final double CHANGE_EPSILON = 1.0e-5;
   private static final String WORLD_BODY_END = "</worldbody>";

   private final RDXMujocoEnvironment environment;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final Set<String> warnedUnsupportedTypes = new HashSet<>();
   private final Set<String> warnedMissingHulls = new HashSet<>();
   private final RigidBodyTransform collisionToWorld = new RigidBodyTransform();
   private final Quaternion quaternion = new Quaternion();
   private final Quaternion meshRotation = new Quaternion();
   private final Vector3D meshOffset = new Vector3D();

   private final AtomicReference<List<RDXArticulatedDoorObject.Dynamics>> doorCommands = new AtomicReference<>(List.of());
   private final Map<String, double[]> simulatedDoorJoints = new ConcurrentHashMap<>();
   private final Map<String, RDXArticulatedDoorObject.JointIds> doorIds = new HashMap<>();

   private MujocoPhysicsEngine physicsEngine;
   private String baseWorldXml;
   private File worldXmlFile;
   private String compiledStructureKey = "";
   private int missingGeomRetries;
   private boolean physicsSyncBroken;
   private int preservedNq = -1;
   private int preservedNv = -1;

   public RDXMujocoObjectCollisionSync(RDXMujocoEnvironment environment)
   {
      this.environment = environment;
   }

   public void attach(SCS2AvatarSimulation simulation)
   {
      PhysicsEngine engine = simulation.getSimulationConstructionSet().getPhysicsEngine();
      if (!(engine instanceof MujocoPhysicsEngine mujocoPhysicsEngine))
      {
         LogTools.error("RDX object collisions were not attached: the simulation is not running MuJoCo");
         return;
      }
      physicsEngine = mujocoPhysicsEngine;
      simulation.getRobot().getControllerManager().addController(this);
   }

   /**
    * Read the builder on the UI thread and publish the collision list the physics thread and the
    * collision visualizer both consume.
    */
   public void syncFrom(RDXEnvironmentBuilder environmentBuilder)
   {
      List<RDXMujocoCollisionShape> shapes = new ArrayList<>();
      List<RDXArticulatedDoorObject.Dynamics> doors = new ArrayList<>();
      for (RDXEnvironmentObject object : environmentBuilder.getAllObjects())
      {
         if (object instanceof RDXArticulatedDoorObject door)
         {
            applySimulatedDoorJoints(door);
            doors.add(door.captureDynamics());
            continue;
         }
         RDXMujocoCollisionShape shape = toCollisionShape(object);
         if (shape != null)
            shapes.add(shape);
      }
      doorCommands.set(List.copyOf(doors));

      List<RDXMujocoCollisionShape> current = environment.getObjectCollisions();
      if (same(current, shapes))
         return;
      environment.setObjectCollisions(shapes);
   }

   @Override
   public void doControl()
   {
      if (physicsEngine == null || physicsSyncBroken)
         return;

      List<RDXMujocoCollisionShape> shapes = environment.getObjectCollisions();
      List<RDXArticulatedDoorObject.Dynamics> doors = doorCommands.get();
      String structureKey = structureKey(shapes, doors);
      MujocoMultiBodyDynamicsWorld world = physicsEngine.getDynamicsWorld();
      mjModel model = world.getModel();
      if (model == null || model.isNull())
         return;

      if (!structureKey.equals(compiledStructureKey))
         recompile(world, shapes, doors, structureKey);
      else
      {
         writePoses(model, shapes);
         syncDynamicDoors(model, world.getData(), doors, false);
      }
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   private void recompile(MujocoMultiBodyDynamicsWorld world,
                          List<RDXMujocoCollisionShape> shapes,
                          List<RDXArticulatedDoorObject.Dynamics> doors,
                          String structureKey)
   {
      try
      {
         if (baseWorldXml == null && !loadBaseWorldXml())
            return;

         copyMeshAssets(shapes, doors);
         String xml = injectObjectGeoms(baseWorldXml, shapes, doors);
         mjModel model = world.getModel();
         mjData data = world.getData();
         int nq = Math.toIntExact(model.nq());
         int nv = Math.toIntExact(model.nv());
         if (preservedNq < 0)
         {
            preservedNq = nq;
            preservedNv = nv;
         }
         double[] qpos = copy(data.qpos(), nq);
         double[] qvel = copy(data.qvel(), nv);
         double time = data.time();
         DoublePointer gravity = model.opt().gravity();
         double gravityX = gravity.get(0);
         double gravityY = gravity.get(1);
         double gravityZ = gravity.get(2);
         double timestep = world.getTimestep();

         world.dispose();
         world.compile(xml, worldXmlFile);

         mjModel recompiled = world.getModel();
         mjData recompiledData = world.getData();
         int expectedNq = preservedNq + 2 * doors.size();
         int expectedNv = preservedNv + 2 * doors.size();
         if (recompiled.nq() != expectedNq || recompiled.nv() != expectedNv)
         {
            physicsSyncBroken = true;
            LogTools.error("MuJoCo recompile nq/nv " + recompiled.nq() + "/" + recompiled.nv()
                           + " did not match the robot plus " + doors.size() + " door(s) (" + expectedNq + "/" + expectedNv
                           + "). Robot state was not restored.");
            return;
         }

         pastePrefix(recompiledData.qpos(), qpos, preservedNq, expectedNq);
         pastePrefix(recompiledData.qvel(), qvel, preservedNv, expectedNv);
         recompiledData.time(time);
         recompiled.opt().gravity().put(0, gravityX);
         recompiled.opt().gravity().put(1, gravityY);
         recompiled.opt().gravity().put(2, gravityZ);
         world.setTimestep(timestep);
         world.writeOptions(physicsEngine.getOptions());
         doorIds.clear();
         syncDynamicDoors(recompiled, recompiledData, doors, true);
         Mujoco.mj_forward(recompiled, recompiledData);
         rebindDiagnostics(world);
         compiledStructureKey = structureKey;
         missingGeomRetries = 0;
         LogTools.info("MuJoCo environment now includes {} RDX object collision(s) and {} dynamic door(s)", shapes.size(), doors.size());
      }
      catch (RuntimeException | IOException exception)
      {
         physicsSyncBroken = true;
         LogTools.error("Failed to add RDX object collisions to MuJoCo: {}", exception.getMessage());
      }
   }

   private void writePoses(mjModel model, List<RDXMujocoCollisionShape> shapes)
   {
      boolean missingGeom = false;
      for (RDXMujocoCollisionShape shape : shapes)
      {
         int geomId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_GEOM, shape.getName());
         if (geomId < 0)
         {
            missingGeom = true;
            continue;
         }
         Orientation3DReadOnly orientation = shape.getPoseInWorld().getOrientation();
         quaternion.set(orientation);
         double x = shape.getPoseInWorld().getPosition().getX();
         double y = shape.getPoseInWorld().getPosition().getY();
         double z = shape.getPoseInWorld().getPosition().getZ();
         // MuJoCo recenters a mesh into its inertia frame and stores that shift in mesh_pos/mesh_quat.
         // The geom pose has to include it, or the collision sits away from the visible mesh.
         if (shape.getType() == RDXMujocoCollisionShape.Type.MESH)
         {
            int meshId = model.geom_dataid().get(geomId);
            if (meshId >= 0)
            {
               meshOffset.set(model.mesh_pos().get(meshId * 3L),
                              model.mesh_pos().get(meshId * 3L + 1),
                              model.mesh_pos().get(meshId * 3L + 2));
               quaternion.transform(meshOffset);
               x += meshOffset.getX();
               y += meshOffset.getY();
               z += meshOffset.getZ();
               meshRotation.set(model.mesh_quat().get(meshId * 4L + 1),
                                model.mesh_quat().get(meshId * 4L + 2),
                                model.mesh_quat().get(meshId * 4L + 3),
                                model.mesh_quat().get(meshId * 4L));
               quaternion.multiply(meshRotation);
            }
         }
         DoublePointer position = model.geom_pos();
         position.put(geomId * 3L, x);
         position.put(geomId * 3L + 1, y);
         position.put(geomId * 3L + 2, z);
         DoublePointer quat = model.geom_quat();
         quat.put(geomId * 4L, quaternion.getS());
         quat.put(geomId * 4L + 1, quaternion.getX());
         quat.put(geomId * 4L + 2, quaternion.getY());
         quat.put(geomId * 4L + 3, quaternion.getZ());
      }
      if (!missingGeom)
      {
         missingGeomRetries = 0;
         return;
      }
      if (missingGeomRetries++ > 2)
      {
         physicsSyncBroken = true;
         LogTools.error("RDX collision geoms are still missing from MuJoCo after rebuild");
         return;
      }
      compiledStructureKey = "";
   }

   private boolean loadBaseWorldXml()
   {
      try
      {
         Field workingDirectoryField = MujocoPhysicsEngine.class.getDeclaredField("workingDirectory");
         workingDirectoryField.setAccessible(true);
         File workingDirectory = (File) workingDirectoryField.get(physicsEngine);
         if (workingDirectory == null)
            return false;
         worldXmlFile = new File(workingDirectory, "world.xml");
         if (!worldXmlFile.isFile())
            return false;
         baseWorldXml = Files.readString(worldXmlFile.toPath());
         return baseWorldXml.contains(WORLD_BODY_END);
      }
      catch (ReflectiveOperationException | IOException exception)
      {
         physicsSyncBroken = true;
         LogTools.error("Could not read the MuJoCo world XML to add RDX object collisions: {}", exception.getMessage());
         return false;
      }
   }

   private void copyMeshAssets(List<RDXMujocoCollisionShape> shapes, List<RDXArticulatedDoorObject.Dynamics> doors) throws IOException
   {
      File workingDirectory = worldXmlFile.getParentFile();
      for (RDXMujocoCollisionShape shape : shapes)
      {
         if (shape.getType() != RDXMujocoCollisionShape.Type.MESH)
            continue;
         copyMesh(workingDirectory, shape.getMeshResourcePath(), shape.getMeshFileName());
      }
      for (RDXArticulatedDoorObject.Dynamics door : doors)
      {
         copyMesh(workingDirectory, door.getPanelMeshPath(), door.panelMeshFileName());
         copyMesh(workingDirectory, door.getLeverMeshPath(), door.leverMeshFileName());
      }
   }

   private static void copyMesh(File workingDirectory, String sourcePath, String fileName) throws IOException
   {
      if (sourcePath == null)
         return;
      File source = new File(sourcePath);
      if (!source.isFile())
         throw new IOException("Convex collision mesh is missing: " + sourcePath);
      Files.copy(source.toPath(), new File(workingDirectory, fileName).toPath(), StandardCopyOption.REPLACE_EXISTING);
   }

   private static String injectObjectGeoms(String baseXml,
                                           List<RDXMujocoCollisionShape> shapes,
                                           List<RDXArticulatedDoorObject.Dynamics> doors)
   {
      String xml = injectMeshAssets(baseXml, shapes, doors);
      int end = xml.lastIndexOf(WORLD_BODY_END);
      if (end < 0)
         throw new IllegalStateException("MuJoCo world XML has no </worldbody>");

      StringBuilder geoms = new StringBuilder();
      if (!shapes.isEmpty())
      {
         geoms.append("    <!-- rdx-environment-collisions -->\n");
         for (RDXMujocoCollisionShape shape : shapes)
            geoms.append(toGeomXml(shape));
         geoms.append("    <!-- /rdx-environment-collisions -->\n");
      }
      if (!doors.isEmpty())
      {
         geoms.append("    <!-- rdx-articulated-doors -->\n");
         for (RDXArticulatedDoorObject.Dynamics door : doors)
            door.appendBodies(geoms);
         geoms.append("    <!-- /rdx-articulated-doors -->\n");
      }
      xml = xml.substring(0, end) + geoms + xml.substring(end);
      if (doors.isEmpty())
         return xml;

      StringBuilder contact = new StringBuilder();
      contact.append("  <contact>\n");
      for (RDXArticulatedDoorObject.Dynamics door : doors)
         door.appendExcludes(contact);
      contact.append("  </contact>\n");
      int mujocoEnd = xml.lastIndexOf("</mujoco>");
      if (mujocoEnd < 0)
         throw new IllegalStateException("MuJoCo world XML has no </mujoco>");
      return xml.substring(0, mujocoEnd) + contact + xml.substring(mujocoEnd);
   }

   private static String injectMeshAssets(String baseXml,
                                          List<RDXMujocoCollisionShape> shapes,
                                          List<RDXArticulatedDoorObject.Dynamics> doors)
   {
      StringBuilder assets = new StringBuilder();
      for (RDXMujocoCollisionShape shape : shapes)
      {
         if (shape.getType() != RDXMujocoCollisionShape.Type.MESH)
            continue;
         assets.append("    <mesh name=\"").append(shape.getName()).append("_mesh\" file=\"")
               .append(shape.getMeshFileName()).append("\"/>\n");
      }
      for (RDXArticulatedDoorObject.Dynamics door : doors)
         door.appendAssets(assets);
      if (assets.isEmpty())
         return baseXml;

      int assetEnd = baseXml.lastIndexOf("</asset>");
      if (assetEnd >= 0)
         return baseXml.substring(0, assetEnd) + assets + baseXml.substring(assetEnd);

      int worldBody = baseXml.indexOf("<worldbody");
      if (worldBody < 0)
         throw new IllegalStateException("MuJoCo world XML has no <worldbody>");
      return baseXml.substring(0, worldBody) + "  <asset>\n" + assets + "  </asset>\n" + baseXml.substring(worldBody);
   }

   private static String toGeomXml(RDXMujocoCollisionShape shape)
   {
      Quaternion rotation = new Quaternion();
      rotation.set(shape.getPoseInWorld().getOrientation());
      String pose = "pos=\"" + fmt(shape.getPoseInWorld().getPosition().getX()) + " "
                    + fmt(shape.getPoseInWorld().getPosition().getY()) + " "
                    + fmt(shape.getPoseInWorld().getPosition().getZ()) + "\" quat=\""
                    + fmt(rotation.getS()) + " " + fmt(rotation.getX()) + " "
                    + fmt(rotation.getY()) + " " + fmt(rotation.getZ()) + "\"";
      if (shape.getType() == RDXMujocoCollisionShape.Type.MESH)
      {
         return "    <geom class=\"terrain\" name=\"" + shape.getName() + "\" " + pose
                + " type=\"mesh\" mesh=\"" + shape.getName() + "_mesh\"/>\n";
      }
      if (shape.getType() == RDXMujocoCollisionShape.Type.SPHERE)
      {
         return "    <geom class=\"terrain\" name=\"" + shape.getName() + "\" " + pose
                + " type=\"sphere\" size=\"" + fmt(shape.getSizeX()) + "\"/>\n";
      }
      return "    <geom class=\"terrain\" name=\"" + shape.getName() + "\" " + pose
             + " type=\"box\" size=\"" + fmt(shape.getSizeX() / 2.0) + " "
             + fmt(shape.getSizeY() / 2.0) + " " + fmt(shape.getSizeZ() / 2.0) + "\"/>\n";
   }

   private void applySimulatedDoorJoints(RDXArticulatedDoorObject door)
   {
      if (door.isJointPositionOverridden())
         return;
      String baseName = "rdx_" + door.getPascalCasedName() + "_" + door.getObjectIndex();
      double[] angles = simulatedDoorJoints.get(baseName);
      if (angles == null)
         return;
      if (Math.abs(door.getHingeAngle() - angles[0]) > CHANGE_EPSILON)
         door.setHingeAngleFromInteraction(angles[0]);
      if (Math.abs(door.getLeverAngle() - angles[1]) > CHANGE_EPSILON)
         door.setLeverAngleFromInteraction(angles[1]);
   }

   private void syncDynamicDoors(mjModel model,
                                 mjData data,
                                 List<RDXArticulatedDoorObject.Dynamics> doors,
                                 boolean seedFromCommand)
   {
      for (RDXArticulatedDoorObject.Dynamics door : doors)
      {
         RDXArticulatedDoorObject.JointIds ids = doorIds.get(door.getBaseName());
         if (ids == null)
         {
            ids = RDXArticulatedDoorObject.JointIds.resolve(model, door.getBaseName());
            if (ids == null)
            {
               if (seedFromCommand)
                  LogTools.warn("Dynamic door {} is missing from the MuJoCo model", door.getBaseName());
               continue;
            }
            doorIds.put(door.getBaseName(), ids);
         }
         simulatedDoorJoints.put(door.getBaseName(), door.syncJoints(model, data, ids, seedFromCommand));
      }
   }

   private RDXMujocoCollisionShape toCollisionShape(RDXEnvironmentObject object)
   {
      Shape3DBasics geometry = object.getCollisionGeometryObject();
      if (geometry == null)
         return null;

      String name = "rdx_" + object.getPascalCasedName() + "_" + object.getObjectIndex();
      File convexMesh = object.getConvexCollisionMeshFile();
      if (convexMesh != null)
      {
         collisionToWorld.set(object.getObjectTransform());
         collisionToWorld.multiply(object.getRealisticModelOffset());
         return new RDXMujocoCollisionShape(name, convexMesh.getAbsolutePath(), new us.ihmc.euclid.geometry.Pose3D(collisionToWorld));
      }
      String convexResourcePath = RDXEnvironmentObject.convexCollisionResourcePath(object.getVisualResourcePath());
      if (convexResourcePath != null && warnedMissingHulls.add(convexResourcePath))
         LogTools.info("No convex hull for {}. Using the primitive collision.", object.getVisualResourcePath());

      collisionToWorld.set(object.getObjectTransform());
      collisionToWorld.multiply(object.getCollisionShapeOffset());
      if (geometry instanceof Box3D box)
      {
         return new RDXMujocoCollisionShape(name,
                                                RDXMujocoCollisionShape.Type.BOX,
                                                box.getSizeX(),
                                                box.getSizeY(),
                                                box.getSizeZ(),
                                                new us.ihmc.euclid.geometry.Pose3D(collisionToWorld));
      }
      if (geometry instanceof Sphere3D sphere)
      {
         return new RDXMujocoCollisionShape(name,
                                                RDXMujocoCollisionShape.Type.SPHERE,
                                                sphere.getRadius(),
                                                sphere.getRadius(),
                                                sphere.getRadius(),
                                                new us.ihmc.euclid.geometry.Pose3D(collisionToWorld));
      }

      String typeName = geometry.getClass().getSimpleName();
      if (warnedUnsupportedTypes.add(typeName))
         LogTools.warn("RDX collision type {} is not copied into MuJoCo. Supported types are convex mesh, box, and sphere.", typeName);
      return null;
   }

   private void rebindDiagnostics(MujocoMultiBodyDynamicsWorld world)
   {
      try
      {
         bind(physicsEngine, "statistics", world);
         bind(physicsEngine, "contactPool", world);
      }
      catch (ReflectiveOperationException exception)
      {
         LogTools.warn("MuJoCo diagnostics were not rebound after adding RDX collisions: {}", exception.getMessage());
      }
   }

   private static void bind(MujocoPhysicsEngine engine, String fieldName, MujocoMultiBodyDynamicsWorld world) throws ReflectiveOperationException
   {
      Field field = MujocoPhysicsEngine.class.getDeclaredField(fieldName);
      field.setAccessible(true);
      Object diagnostics = field.get(engine);
      if (diagnostics == null)
         return;
      diagnostics.getClass().getMethod("bind", mjModel.class, mjData.class).invoke(diagnostics, world.getModel(), world.getData());
   }

   private static boolean same(List<RDXMujocoCollisionShape> current, List<RDXMujocoCollisionShape> next)
   {
      if (current.size() != next.size())
         return false;
      for (int i = 0; i < current.size(); i++)
      {
         if (!current.get(i).matches(next.get(i), CHANGE_EPSILON))
            return false;
      }
      return true;
   }

   private static String structureKey(List<RDXMujocoCollisionShape> shapes, List<RDXArticulatedDoorObject.Dynamics> doors)
   {
      StringBuilder key = new StringBuilder();
      for (RDXMujocoCollisionShape shape : shapes)
         key.append(shape.structureKey()).append('\n');
      for (RDXArticulatedDoorObject.Dynamics door : doors)
         key.append(door.structureKey()).append('\n');
      return key.toString();
   }

   private static double[] copy(DoublePointer pointer, int length)
   {
      double[] values = new double[length];
      for (int i = 0; i < length; i++)
         values[i] = pointer.get(i);
      return values;
   }

   private static void pastePrefix(DoublePointer pointer, double[] values, int keep, int total)
   {
      int count = Math.min(keep, values.length);
      for (int i = 0; i < count; i++)
         pointer.put(i, values[i]);
      for (int i = count; i < total; i++)
         pointer.put(i, 0.0);
   }

   private static String fmt(double value)
   {
      return String.format(Locale.US, "%.8g", value);
   }
}
