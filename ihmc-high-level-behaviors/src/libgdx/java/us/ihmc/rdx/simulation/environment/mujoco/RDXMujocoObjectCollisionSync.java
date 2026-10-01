package us.ihmc.rdx.simulation.environment.mujoco;

import org.bytedeco.javacpp.DoublePointer;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
import us.ihmc.scs2.simulation.robot.Robot;
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
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Copies RDX environment-builder collisions into the running MuJoCo model.
 * <p>
 * When a visual GLB has a precomputed {@code <stem>_convex.stl} beside it, that hull is the
 * collision mesh. Objects without one keep their box or sphere. An articulated door is not a
 * static geom: it is a dynamic mechanism (welded frame, hinged panel, sprung lever). Adding or
 * removing an object rebuilds the world XML and recompiles, keeping the robot state. Dragging a
 * static object only writes that mocap pose. Dragging a door writes the frame body pose.
 */
public class RDXMujocoObjectCollisionSync implements Controller
{
   private static final double CHANGE_EPSILON = 1.0e-5;
   private static final String WORLD_BODY_END = "</worldbody>";
   /** Factory geom names are {@code terrain_<terrainIndex>_<shapeIndex>}. */
   private static final Pattern FACTORY_TERRAIN_GEOM = Pattern.compile("name=\"terrain_(\\d+)_(\\d+)\"");

   private final RDXMujocoEnvironment environment;
   private final int groundCollisionShapeCount;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final Set<String> warnedUnsupportedTypes = new HashSet<>();
   private final Set<String> warnedMissingHulls = new HashSet<>();
   private final RigidBodyTransform collisionToWorld = new RigidBodyTransform();
   private final Quaternion quaternion = new Quaternion();

   private final AtomicReference<List<RDXArticulatedDoorObject.Dynamics>> doorCommands = new AtomicReference<>(List.of());
   private final Map<String, double[]> simulatedDoorJoints = new ConcurrentHashMap<>();
   private final Map<String, RDXArticulatedDoorObject.JointIds> doorIds = new HashMap<>();

   private MujocoPhysicsEngine physicsEngine;
   private Robot robot;
   private RDXMujocoIgnoredJointCollisions ignoredJointCollisions = RDXMujocoIgnoredJointCollisions.none();
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
      this.groundCollisionShapeCount = environment.getGroundCollisionShapeCount();
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
      robot = simulation.getRobot();
      ignoredJointCollisions = RDXMujocoIgnoredJointCollisions.fromRobot(robot);
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
      if (!ignoredJointCollisions.isEmpty())
         structureKey = structureKey + "hands\n";
      MujocoMultiBodyDynamicsWorld world = physicsEngine.getDynamicsWorld();
      mjModel model = world.getModel();
      if (model == null || model.isNull())
         return;

      if (!structureKey.equals(compiledStructureKey))
         recompile(world, shapes, doors, structureKey);
      else
      {
         writePoses(model, world.getData(), shapes);
         ignoredJointCollisions.writePoses(robot, model, world.getData());
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
         // The mesh frame is only known after compiling, so place the collisions before the first
         // step rather than leaving them one tick off the visual.
         writePoses(recompiled, recompiledData, shapes);
         syncDynamicDoors(recompiled, recompiledData, doors, true);
         Mujoco.mj_forward(recompiled, recompiledData);
         rebindDiagnostics(world);
         compiledStructureKey = structureKey;
         ignoredJointCollisions.writePoses(robot, recompiled, recompiledData);
         missingGeomRetries = 0;
         LogTools.info("MuJoCo environment now includes {} RDX object collision(s) and {} dynamic door(s)", shapes.size(), doors.size());
      }
      catch (RuntimeException | IOException exception)
      {
         physicsSyncBroken = true;
         LogTools.error("Failed to add RDX object collisions to MuJoCo: {}", exception.getMessage());
      }
   }

   private void writePoses(mjModel model, mjData data, List<RDXMujocoCollisionShape> shapes)
   {
      boolean missingGeom = false;
      for (RDXMujocoCollisionShape shape : shapes)
      {
         int bodyId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_BODY, shape.getBodyName());
         int geomId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_GEOM, shape.getName());
         int mocapId = bodyId < 0 ? -1 : model.body_mocapid().get(bodyId);
         if (bodyId < 0 || geomId < 0 || mocapId < 0)
         {
            missingGeom = true;
            continue;
         }
         Orientation3DReadOnly orientation = shape.getPoseInWorld().getOrientation();
         quaternion.set(orientation);
         double x = shape.getPoseInWorld().getPosition().getX();
         double y = shape.getPoseInWorld().getPosition().getY();
         double z = shape.getPoseInWorld().getPosition().getZ();
         // Leave mesh_pos/mesh_quat alone. MuJoCo already applies that inertia-frame shift
         // when it places the geom, so writing it into the mocap pose rotates the collision
         // off the visual. The table's shift is a 120 degree axis swap.
         DoublePointer position = data.mocap_pos();
         position.put(mocapId * 3L, x);
         position.put(mocapId * 3L + 1, y);
         position.put(mocapId * 3L + 2, z);
         DoublePointer quat = data.mocap_quat();
         quat.put(mocapId * 4L, quaternion.getS());
         quat.put(mocapId * 4L + 1, quaternion.getX());
         quat.put(mocapId * 4L + 2, quaternion.getY());
         quat.put(mocapId * 4L + 3, quaternion.getZ());
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

   /**
    * The first compile names loaded boxes {@code terrain_0_<n>} after the ground geoms. Drop those
    * before injecting {@code rdx_*} geoms so a later rebuild does not stack a second copy.
    */
   private String withoutBakedObjectGeoms(String xml)
   {
      String[] lines = xml.split("\\R", -1);
      StringBuilder out = new StringBuilder();
      for (int i = 0; i < lines.length; i++)
      {
         Matcher matcher = FACTORY_TERRAIN_GEOM.matcher(lines[i]);
         if (matcher.find() && Integer.parseInt(matcher.group(1)) == 0 && Integer.parseInt(matcher.group(2)) >= groundCollisionShapeCount)
            continue;
         if (i > 0)
            out.append('\n');
         out.append(lines[i]);
      }
      return out.toString();
   }

   private String injectObjectGeoms(String baseXml,
                                    List<RDXMujocoCollisionShape> shapes,
                                    List<RDXArticulatedDoorObject.Dynamics> doors)
   {
      String xml = injectMeshAssets(withoutBakedObjectGeoms(baseXml), shapes, doors);
      int end = xml.lastIndexOf(WORLD_BODY_END);
      if (end < 0)
         throw new IllegalStateException("MuJoCo world XML has no </worldbody>");

      StringBuilder geoms = new StringBuilder();
      if (!shapes.isEmpty())
      {
         geoms.append("    <!-- rdx-environment-collisions -->\n");
         for (RDXMujocoCollisionShape shape : shapes)
            geoms.append(toBodyXml(shape));
         geoms.append("    <!-- /rdx-environment-collisions -->\n");
      }
      if (!doors.isEmpty())
      {
         geoms.append("    <!-- rdx-articulated-doors -->\n");
         for (RDXArticulatedDoorObject.Dynamics door : doors)
            door.appendBodies(geoms);
         geoms.append("    <!-- /rdx-articulated-doors -->\n");
      }
      xml = xml.substring(0, end) + geoms + ignoredJointCollisions.bodyXml(robot.getRobotDefinition()) + xml.substring(end);
      xml = insertBefore(xml, "</contact>", ignoredJointCollisions.excludeXml());
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

   private static String insertBefore(String xml, String marker, String insertion)
   {
      if (insertion.isEmpty())
         return xml;
      int at = xml.indexOf(marker);
      if (at < 0)
         return xml;
      return xml.substring(0, at) + insertion + xml.substring(at);
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

   private static String toBodyXml(RDXMujocoCollisionShape shape)
   {
      Quaternion rotation = new Quaternion();
      rotation.set(shape.getPoseInWorld().getOrientation());
      String pose = "pos=\"" + fmt(shape.getPoseInWorld().getPosition().getX()) + " "
                    + fmt(shape.getPoseInWorld().getPosition().getY()) + " "
                    + fmt(shape.getPoseInWorld().getPosition().getZ()) + "\" quat=\""
                    + fmt(rotation.getS()) + " " + fmt(rotation.getX()) + " "
                    + fmt(rotation.getY()) + " " + fmt(rotation.getZ()) + "\"";
      String geometry;
      if (shape.getType() == RDXMujocoCollisionShape.Type.MESH)
         geometry = "type=\"mesh\" mesh=\"" + shape.getName() + "_mesh\"";
      else if (shape.getType() == RDXMujocoCollisionShape.Type.SPHERE)
         geometry = "type=\"sphere\" size=\"" + fmt(shape.getSizeX()) + "\"";
      else
         geometry = "type=\"box\" size=\"" + fmt(shape.getSizeX() / 2.0) + " "
                    + fmt(shape.getSizeY() / 2.0) + " " + fmt(shape.getSizeZ() / 2.0) + "\"";

      // A child of worldbody with no joint becomes a free joint, which changes nq and lets the
      // object fall. mocap keeps it fixed for the solver while writePoses can still move it, and
      // the broadphase box follows that pose. A plain worldbody geom would not.
      return "    <body name=\"" + shape.getBodyName() + "\" mocap=\"true\" " + pose + ">\n"
             + "      <geom class=\"terrain\" name=\"" + shape.getName() + "\" " + geometry + "/>\n"
             + "    </body>\n";
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
