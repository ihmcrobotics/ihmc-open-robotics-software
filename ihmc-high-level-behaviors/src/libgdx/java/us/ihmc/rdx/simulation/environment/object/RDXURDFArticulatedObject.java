package us.ihmc.rdx.simulation.environment.object;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.math.collision.BoundingBox;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import jakarta.xml.bind.JAXBException;
import org.bytedeco.javacpp.DoublePointer;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.simulation.environment.mujoco.RDXMujocoArticulatedDynamics;
import us.ihmc.rdx.simulation.environment.mujoco.RDXMujocoArticulatedObject;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.scs2.definition.robot.urdf.URDFTools;
import us.ihmc.scs2.definition.robot.urdf.items.URDFCollision;
import us.ihmc.scs2.definition.robot.urdf.items.URDFDynamics;
import us.ihmc.scs2.definition.robot.urdf.items.URDFGeometry;
import us.ihmc.scs2.definition.robot.urdf.items.URDFInertial;
import us.ihmc.scs2.definition.robot.urdf.items.URDFJoint;
import us.ihmc.scs2.definition.robot.urdf.items.URDFLimit;
import us.ihmc.scs2.definition.robot.urdf.items.URDFLink;
import us.ihmc.scs2.definition.robot.urdf.items.URDFModel;
import us.ihmc.scs2.definition.robot.urdf.items.URDFOrigin;
import us.ihmc.scs2.definition.robot.urdf.items.URDFVisual;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;

import java.io.File;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * An environment object built from a URDF: one rigid part per link, connected by the URDF's
 * revolute, continuous, prismatic and fixed joints. The root link is the object's placement frame
 * and is welded to the world in MuJoCo, like the articulated door's frame.
 * <p>
 * Every visual and collision is a mesh file, resolved relative to the URDF.
 * {@code scripts/add_rdx_articulated_object_from_urdf.py} writes that layout: {@code <part>.glb}
 * visuals, {@code <part>_collision_<i>.stl} convex collision pieces, and
 * {@code <stem>_rdx.json} beside the URDF, whose {@code excludeContacts} lists body pairs MuJoCo
 * should not collide. A subclass only names the object and its URDF resource.
 * <p>
 * Joint positions come from the simulation, or from the sliders while "Override joint positions"
 * is checked.
 */
public class RDXURDFArticulatedObject extends RDXEnvironmentObject implements RDXMujocoArticulatedObject
{
   /** N m s/rad or N s/m, used when the URDF gives no damping. The articulated door's hinge uses the same. */
   private static final double DEFAULT_DAMPING = 2.0;
   /** Kg m^2 or kg. Keeps light parts from chattering under a gripping hand, as on the door lever. */
   private static final double ARMATURE = 0.01;
   private static final double MIN_MASS = 1.0e-3;
   private static final double MIN_INERTIA = 1.0e-6;
   private static final double CHANGE_EPSILON = 1.0e-5;

   private final String urdfResourcePath;
   /** Root first, then each part after its parent. */
   private final List<PartDefinition> parts;
   private final List<String[]> excludedBodyPairs;
   private final double[] jointPositions;
   private final RigidBodyTransform[] partToObject;
   private final List<List<PlacedMesh>> visualInstances = new ArrayList<>();
   private final List<List<PlacedMesh>> collisionInstances = new ArrayList<>();

   private final ImBoolean overrideJointPositions = new ImBoolean(false);
   private boolean wasOverridingJointPositions;
   private final float[] sliderValue = new float[1];
   private final RigidBodyTransform jointMotion = new RigidBodyTransform();
   private final RigidBodyTransform meshToWorld = new RigidBodyTransform();
   private final AxisAngle axisAngle = new AxisAngle();

   public RDXURDFArticulatedObject(String titleCasedName, RDXEnvironmentObjectFactory factory, String urdfResourcePath)
   {
      super(titleCasedName, factory);
      this.urdfResourcePath = urdfResourcePath;

      File urdfFile = resolveResourceFile(urdfResourcePath);
      if (urdfFile == null)
         throw new IllegalArgumentException("URDF resource not found: " + urdfResourcePath);
      parts = Collections.unmodifiableList(loadParts(urdfFile));
      excludedBodyPairs = Collections.unmodifiableList(loadExcludedBodyPairs(urdfFile, parts));
      jointPositions = new double[parts.size()];
      partToObject = new RigidBodyTransform[parts.size()];
      for (int i = 0; i < parts.size(); i++)
      {
         partToObject[i] = new RigidBodyTransform();
         jointPositions[i] = parts.get(i).clamp(0.0);
      }

      loadGraphics();
      updatePartTransforms();
      setUpPickingBox();
      updateRenderablesPoses();
   }

   // -------------------------------------------------------------------------------------------
   // URDF
   // -------------------------------------------------------------------------------------------

   private static List<PartDefinition> loadParts(File urdfFile)
   {
      File directory = urdfFile.getAbsoluteFile().getParentFile();
      URDFModel model;
      try
      {
         // Mesh names are relative to the URDF. URDFTools replaces any name it cannot resolve with null,
         // so give it the URDF's folder. It may append to this list, so it must be mutable.
         model = URDFTools.loadURDFModel(urdfFile, new ArrayList<>(List.of(directory.getAbsolutePath())));
      }
      catch (JAXBException exception)
      {
         throw new IllegalArgumentException("Could not parse " + urdfFile + ": " + exception.getMessage(), exception);
      }

      Map<String, URDFLink> links = new LinkedHashMap<>();
      for (URDFLink link : nonNull(model.getLinks()))
         links.put(link.getName(), link);
      Map<String, List<URDFJoint>> jointsByParent = new HashMap<>();
      Map<String, URDFJoint> jointByChild = new HashMap<>();
      for (URDFJoint joint : nonNull(model.getJoints()))
      {
         String parent = joint.getParent().getLink();
         String child = joint.getChild().getLink();
         if (!links.containsKey(parent) || !links.containsKey(child))
            throw new IllegalArgumentException("Joint " + joint.getName() + " names a link that is not in " + urdfFile.getName());
         if (jointByChild.put(child, joint) != null)
            throw new IllegalArgumentException("Link " + child + " is the child of two joints in " + urdfFile.getName());
         jointsByParent.computeIfAbsent(parent, key -> new ArrayList<>()).add(joint);
      }
      List<String> roots = links.keySet().stream().filter(name -> !jointByChild.containsKey(name)).toList();
      if (roots.size() != 1)
         throw new IllegalArgumentException("Expected one root link in " + urdfFile.getName() + ", found " + roots);

      List<PartDefinition> parts = new ArrayList<>();
      Map<String, Integer> indexByLink = new HashMap<>();
      Deque<String> frontier = new ArrayDeque<>(roots);
      while (!frontier.isEmpty())
      {
         String name = frontier.removeFirst();
         URDFJoint joint = jointByChild.get(name);
         int parentIndex = joint == null ? -1 : indexByLink.get(joint.getParent().getLink());
         indexByLink.put(name, parts.size());
         parts.add(PartDefinition.from(links.get(name), joint, parentIndex, directory));
         for (URDFJoint child : jointsByParent.getOrDefault(name, List.of()))
            frontier.addLast(child.getChild().getLink());
      }
      if (parts.size() != links.size())
         throw new IllegalArgumentException(urdfFile.getName() + " has links that are not connected to the root");
      return parts;
   }

   /**
    * {@code <stem>_rdx.json} beside the URDF. Without it, every part and its parent across a
    * movable joint are excluded, which is the part of that file that does not need MuJoCo to compute.
    */
   private static List<String[]> loadExcludedBodyPairs(File urdfFile, List<PartDefinition> parts)
   {
      String stem = urdfFile.getName().replaceFirst("\\.urdf$", "");
      File sidecar = new File(urdfFile.getParentFile(), stem + "_rdx.json");
      List<String[]> pairs = new ArrayList<>();
      if (sidecar.isFile())
      {
         try
         {
            JsonNode root = new ObjectMapper().readTree(sidecar);
            for (JsonNode pair : root.path("excludeContacts"))
               pairs.add(new String[] {pair.path("body1").asText(), pair.path("body2").asText()});
            return pairs;
         }
         catch (IOException exception)
         {
            LogTools.warn("Could not read {}: {}. Excluding only jointed pairs.", sidecar.getName(), exception.getMessage());
         }
      }
      for (PartDefinition part : parts)
      {
         if (part.parentIndex >= 0 && part.type.isMovable())
            pairs.add(new String[] {parts.get(part.parentIndex).name, part.name});
      }
      return pairs;
   }

   private static <T> List<T> nonNull(List<T> list)
   {
      return list == null ? List.of() : list;
   }

   // -------------------------------------------------------------------------------------------
   // Graphics
   // -------------------------------------------------------------------------------------------

   private void loadGraphics()
   {
      Color collisionColor = LibGDXTools.toLibGDX(YoAppearance.LightSkyBlue());
      for (int i = 0; i < parts.size(); i++)
      {
         PartDefinition part = parts.get(i);
         List<PlacedMesh> visuals = new ArrayList<>();
         for (MeshReference visual : part.visuals)
         {
            Model model = RDXModelLoader.load(visual.file.getAbsolutePath());
            // The root's first visual is the object's realistic model, so the base class places and selects it
            if (i == 0 && realisticModel == null)
            {
               setRealisticModel(model);
               realisticModelOffset.set(visual.origin);
               continue;
            }
            visuals.add(new PlacedMesh(new RDXModelInstance(model), visual.origin));
         }
         visualInstances.add(visuals);

         List<PlacedMesh> collisions = new ArrayList<>();
         for (MeshReference collision : part.collisions)
         {
            RDXModelInstance instance = new RDXModelInstance(RDXModelLoader.load(collision.file.getAbsolutePath()));
            instance.setDiffuseColor(collisionColor);
            instance.setOpacity(0.4f);
            collisions.add(new PlacedMesh(instance, collision.origin));
         }
         collisionInstances.add(collisions);
      }
      if (realisticModel == null)
         throw new IllegalArgumentException(urdfResourcePath + ": the root link " + parts.get(0).name + " needs a visual mesh");
   }

   /** A box around every visual at the starting joint positions, used for picking and the selection outline. */
   private void setUpPickingBox()
   {
      Point3D min = new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point3D max = new Point3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
      expand(realisticModelInstance, realisticModelOffset, min, max);
      for (int i = 0; i < parts.size(); i++)
      {
         for (PlacedMesh visual : visualInstances.get(i))
         {
            RigidBodyTransform meshToObject = new RigidBodyTransform(partToObject[i]);
            meshToObject.multiply(visual.origin);
            expand(visual.instance, meshToObject, min, max);
         }
      }
      Vector3D size = new Vector3D();
      size.sub(max, min);
      Point3D center = new Point3D();
      center.interpolate(min, max, 0.5);

      double totalMass = 0.0;
      for (PartDefinition part : parts)
         totalMass += part.mass;
      setMass((float) totalMass);
      getCollisionShapeOffset().getTranslation().set(center);
      getBoundingSphere().setRadius(size.norm() / 2.0);
      setCollisionModel(meshBuilder -> meshBuilder.addBox((float) size.getX(),
                                                          (float) size.getY(),
                                                          (float) size.getZ(),
                                                          LibGDXTools.toLibGDX(YoAppearance.DarkGray())));
      setCollisionGeometryObject(new Box3D(size.getX(), size.getY(), size.getZ()));
   }

   private static void expand(RDXModelInstance instance, RigidBodyTransformReadOnly meshToObject, Point3D min, Point3D max)
   {
      BoundingBox box = instance.calculateBoundingBox(new BoundingBox());
      Vector3 corner = new Vector3();
      Point3D point = new Point3D();
      for (int i = 0; i < 8; i++)
      {
         corner.set((i & 1) == 0 ? box.min.x : box.max.x, (i & 2) == 0 ? box.min.y : box.max.y, (i & 4) == 0 ? box.min.z : box.max.z);
         point.set(corner.x, corner.y, corner.z);
         meshToObject.transform(point);
         min.set(Math.min(min.getX(), point.getX()), Math.min(min.getY(), point.getY()), Math.min(min.getZ(), point.getZ()));
         max.set(Math.max(max.getX(), point.getX()), Math.max(max.getY(), point.getY()), Math.max(max.getZ(), point.getZ()));
      }
   }

   @Override
   public void getRealRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      super.getRealRenderables(renderables, pool);
      for (List<PlacedMesh> visuals : visualInstances)
         for (PlacedMesh visual : visuals)
            visual.instance.getRenderables(renderables, pool);
   }

   /** The convex pieces MuJoCo collides with, not the picking box. */
   @Override
   public void getCollisionMeshRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (List<PlacedMesh> collisions : collisionInstances)
         for (PlacedMesh collision : collisions)
            collision.instance.getRenderables(renderables, pool);
   }

   @Override
   public void updateRenderablesPoses()
   {
      updatePartTransforms();
      super.updateRenderablesPoses();
      for (int i = 0; i < parts.size(); i++)
      {
         for (PlacedMesh visual : visualInstances.get(i))
            place(visual, i);
         for (PlacedMesh collision : collisionInstances.get(i))
            place(collision, i);
      }
   }

   private void place(PlacedMesh mesh, int partIndex)
   {
      meshToWorld.set(getObjectTransform());
      meshToWorld.multiply(partToObject[partIndex]);
      meshToWorld.multiply(mesh.origin);
      LibGDXTools.toLibGDX(meshToWorld, mesh.instance.transform);
   }

   private void updatePartTransforms()
   {
      for (int i = 0; i < parts.size(); i++)
      {
         PartDefinition part = parts.get(i);
         if (part.parentIndex < 0)
         {
            partToObject[i].setToZero();
            continue;
         }
         partToObject[i].set(partToObject[part.parentIndex]);
         partToObject[i].multiply(part.jointOrigin);
         part.motion(jointPositions[i], axisAngle, jointMotion);
         partToObject[i].multiply(jointMotion);
      }
   }

   // -------------------------------------------------------------------------------------------
   // Joints
   // -------------------------------------------------------------------------------------------

   /** Names of the movable joints, in the order {@link #applySimulatedJointPositions(double[])} expects. */
   public List<String> getJointNames()
   {
      List<String> names = new ArrayList<>();
      for (PartDefinition part : parts)
      {
         if (part.type.isMovable())
            names.add(part.jointName);
      }
      return names;
   }

   public double getJointPosition(String jointName)
   {
      return jointPositions[partIndexOfJoint(jointName)];
   }

   /** Sets a joint position, clamped to its limits, and moves the visuals. */
   public void setJointPosition(String jointName, double position)
   {
      int index = partIndexOfJoint(jointName);
      jointPositions[index] = parts.get(index).clamp(position);
      updateRenderablesPoses();
   }

   public boolean isJointPositionOverridden()
   {
      return overrideJointPositions.get();
   }

   private int partIndexOfJoint(String jointName)
   {
      for (int i = 0; i < parts.size(); i++)
      {
         if (parts.get(i).type.isMovable() && parts.get(i).jointName.equals(jointName))
            return i;
      }
      throw new IllegalArgumentException(titleCasedName + " has no movable joint " + jointName);
   }

   @Override
   public void applySimulatedJointPositions(double[] simulatedPositions)
   {
      if (isJointPositionOverridden())
         return;
      boolean changed = false;
      int movable = 0;
      for (int i = 0; i < parts.size(); i++)
      {
         if (!parts.get(i).type.isMovable())
            continue;
         double position = simulatedPositions[movable++];
         if (Math.abs(jointPositions[i] - position) > CHANGE_EPSILON)
         {
            jointPositions[i] = position;
            changed = true;
         }
      }
      if (changed)
         updateRenderablesPoses();
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.separator();
      ImGui.text("Joint positions");
      String id = "##" + pascalCasedName + objectIndex;
      ImGui.checkbox("Override joint positions" + id, overrideJointPositions);
      boolean overriding = overrideJointPositions.get();
      boolean entering = overriding && !wasOverridingJointPositions;
      wasOverridingJointPositions = overriding;

      if (!overriding)
         ImGui.beginDisabled();
      boolean changed = false;
      for (int i = 0; i < parts.size(); i++)
      {
         PartDefinition part = parts.get(i);
         if (!part.type.isMovable())
            continue;
         sliderValue[0] = (float) jointPositions[i];
         String format = part.type == JointType.PRISMATIC ? "%.3f m" : "%.3f rad";
         if (ImGui.sliderFloat(part.jointName + id + part.jointName, sliderValue, (float) part.sliderLower(), (float) part.sliderUpper(), format)
             && overriding)
         {
            jointPositions[i] = part.clamp(sliderValue[0]);
            changed = true;
         }
      }
      if (!overriding)
         ImGui.endDisabled();
      if (changed || entering)
         updateRenderablesPoses();
   }

   // -------------------------------------------------------------------------------------------
   // MuJoCo
   // -------------------------------------------------------------------------------------------

   @Override
   public Dynamics captureDynamics()
   {
      double[] movablePositions = new double[getJointNames().size()];
      int movable = 0;
      for (int i = 0; i < parts.size(); i++)
      {
         if (parts.get(i).type.isMovable())
            movablePositions[movable++] = jointPositions[i];
      }
      return new Dynamics("rdx_" + pascalCasedName + "_" + objectIndex,
                          urdfResourcePath,
                          parts,
                          excludedBodyPairs,
                          new Pose3D(getObjectTransform()),
                          movablePositions,
                          isJointPositionOverridden());
   }

   /**
    * Physics snapshot of one placed object. The part definitions are shared and never change; the
    * placement and joint positions are copied.
    */
   public static final class Dynamics implements RDXMujocoArticulatedDynamics
   {
      private final String baseName;
      private final String urdfResourcePath;
      private final List<PartDefinition> parts;
      private final List<String[]> excludedBodyPairs;
      private final Pose3D rootPose;
      private final double[] jointPositions;
      private final boolean overrideJoints;

      private Dynamics(String baseName,
                       String urdfResourcePath,
                       List<PartDefinition> parts,
                       List<String[]> excludedBodyPairs,
                       Pose3D rootPose,
                       double[] jointPositions,
                       boolean overrideJoints)
      {
         this.baseName = baseName;
         this.urdfResourcePath = urdfResourcePath;
         this.parts = parts;
         this.excludedBodyPairs = excludedBodyPairs;
         this.rootPose = rootPose;
         this.jointPositions = jointPositions;
         this.overrideJoints = overrideJoints;
      }

      @Override
      public String getBaseName()
      {
         return baseName;
      }

      @Override
      public String structureKey()
      {
         return baseName + "|urdf|" + urdfResourcePath;
      }

      @Override
      public int getDegreesOfFreedom()
      {
         return jointPositions.length;
      }

      @Override
      public Map<String, String> getMeshFiles()
      {
         Map<String, String> files = new LinkedHashMap<>();
         for (PartDefinition part : parts)
         {
            for (int piece = 0; piece < part.collisions.size(); piece++)
               files.put(meshFileName(part, piece), part.collisions.get(piece).file.getAbsolutePath());
         }
         return files;
      }

      private String bodyName(PartDefinition part)
      {
         return baseName + "_" + part.name;
      }

      private String jointName(PartDefinition part)
      {
         return baseName + "_" + part.jointName;
      }

      private String geomName(PartDefinition part, int piece)
      {
         return baseName + "_" + part.name + "_" + piece;
      }

      private String meshFileName(PartDefinition part, int piece)
      {
         String file = part.collisions.get(piece).file.getName();
         return geomName(part, piece) + file.substring(file.lastIndexOf('.'));
      }

      @Override
      public void appendAssets(StringBuilder assets)
      {
         for (PartDefinition part : parts)
         {
            for (int piece = 0; piece < part.collisions.size(); piece++)
            {
               assets.append("    <mesh name=\"").append(geomName(part, piece)).append("_mesh\" file=\"")
                     .append(meshFileName(part, piece)).append("\"/>\n");
            }
         }
      }

      /**
       * Nests one body per part. The root has no joint, so it is welded to the world, and
       * {@link #syncJoints} moves it by writing its model pose, like the door frame.
       */
      @Override
      public void appendBodies(StringBuilder bodies)
      {
         appendBody(bodies, 0, "    ");
      }

      private void appendBody(StringBuilder bodies, int index, String indent)
      {
         PartDefinition part = parts.get(index);
         bodies.append(indent).append("<body name=\"").append(bodyName(part)).append("\" ");
         if (part.parentIndex < 0)
            bodies.append(pose(new RigidBodyTransform(rootPose.getOrientation(), rootPose.getPosition())));
         else
            bodies.append(pose(part.jointOrigin));
         bodies.append(">\n");
         String inner = indent + "  ";
         if (part.type.isMovable())
         {
            bodies.append(inner).append("<joint name=\"").append(jointName(part)).append("\" type=\"")
                  .append(part.type == JointType.PRISMATIC ? "slide" : "hinge").append("\" axis=\"")
                  .append(fmt(part.axis.getX())).append(' ').append(fmt(part.axis.getY())).append(' ').append(fmt(part.axis.getZ())).append('"');
            if (part.type != JointType.CONTINUOUS && Double.isFinite(part.lower) && Double.isFinite(part.upper))
               bodies.append(" limited=\"true\" range=\"").append(fmt(part.lower)).append(' ').append(fmt(part.upper)).append('"');
            bodies.append(" damping=\"").append(fmt(part.damping > 0.0 ? part.damping : DEFAULT_DAMPING)).append('"');
            if (part.friction > 0.0)
               bodies.append(" frictionloss=\"").append(fmt(part.friction)).append('"');
            bodies.append(" armature=\"").append(fmt(ARMATURE)).append("\"/>\n");
         }
         if (part.parentIndex >= 0)
            appendInertial(bodies, inner, part);
         for (int piece = 0; piece < part.collisions.size(); piece++)
         {
            bodies.append(inner).append("<geom class=\"terrain\" name=\"").append(geomName(part, piece))
                  .append("\" type=\"mesh\" mesh=\"").append(geomName(part, piece)).append("_mesh\" ")
                  .append(pose(part.collisions.get(piece).origin)).append("/>\n");
         }
         for (int child = 0; child < parts.size(); child++)
         {
            if (parts.get(child).parentIndex == index)
               appendBody(bodies, child, inner);
         }
         bodies.append(indent).append("</body>\n");
      }

      /** MuJoCo refuses fullinertia together with an inertial orientation, so the inertia is rotated into the body frame here. */
      private static void appendInertial(StringBuilder bodies, String indent, PartDefinition part)
      {
         Matrix3D inertia = new Matrix3D(part.inertia);
         part.inertiaPose.getRotation().transform(inertia);
         boolean valid = part.mass >= MIN_MASS && inertia.getM00() > 0.0 && inertia.getM11() > 0.0 && inertia.getM22() > 0.0;
         double mass = Math.max(part.mass, MIN_MASS);
         bodies.append(indent).append("<inertial pos=\"").append(fmt(part.inertiaPose.getTranslation().getX())).append(' ')
               .append(fmt(part.inertiaPose.getTranslation().getY())).append(' ').append(fmt(part.inertiaPose.getTranslation().getZ()))
               .append("\" mass=\"").append(fmt(mass)).append('"');
         if (valid)
         {
            bodies.append(" fullinertia=\"").append(fmt(inertia.getM00())).append(' ').append(fmt(inertia.getM11())).append(' ')
                  .append(fmt(inertia.getM22())).append(' ').append(fmt(inertia.getM01())).append(' ')
                  .append(fmt(inertia.getM02())).append(' ').append(fmt(inertia.getM12())).append('"');
         }
         else
         {
            bodies.append(" diaginertia=\"").append(fmt(MIN_INERTIA)).append(' ').append(fmt(MIN_INERTIA)).append(' ')
                  .append(fmt(MIN_INERTIA)).append('"');
         }
         bodies.append("/>\n");
      }

      @Override
      public void appendExcludes(StringBuilder contact)
      {
         for (String[] pair : excludedBodyPairs)
         {
            contact.append("    <exclude body1=\"").append(baseName).append('_').append(pair[0])
                   .append("\" body2=\"").append(baseName).append('_').append(pair[1]).append("\"/>\n");
         }
      }

      @Override
      public Object bind(mjModel model)
      {
         int rootBody = Mujoco.mj_name2id(model, Mujoco.mjOBJ_BODY, bodyName(parts.get(0)));
         if (rootBody < 0)
            return null;
         List<int[]> joints = new ArrayList<>();
         for (PartDefinition part : parts)
         {
            if (!part.type.isMovable())
               continue;
            int joint = Mujoco.mj_name2id(model, Mujoco.mjOBJ_JOINT, jointName(part));
            if (joint < 0)
               return null;
            joints.add(new int[] {model.jnt_qposadr().get(joint), model.jnt_dofadr().get(joint)});
         }
         return new Binding(rootBody, joints.toArray(int[][]::new));
      }

      @Override
      public double[] syncJoints(mjModel model, mjData data, Object binding, boolean seedFromCommand)
      {
         Binding ids = (Binding) binding;
         Quaternion rotation = new Quaternion(rootPose.getOrientation());
         DoublePointer position = model.body_pos();
         position.put(ids.rootBody * 3L, rootPose.getX());
         position.put(ids.rootBody * 3L + 1, rootPose.getY());
         position.put(ids.rootBody * 3L + 2, rootPose.getZ());
         DoublePointer quat = model.body_quat();
         quat.put(ids.rootBody * 4L, rotation.getS());
         quat.put(ids.rootBody * 4L + 1, rotation.getX());
         quat.put(ids.rootBody * 4L + 2, rotation.getY());
         quat.put(ids.rootBody * 4L + 3, rotation.getZ());

         double[] simulated = new double[ids.joints.length];
         for (int i = 0; i < ids.joints.length; i++)
         {
            int qpos = ids.joints[i][0];
            int dof = ids.joints[i][1];
            if (seedFromCommand || overrideJoints)
            {
               data.qpos().put(qpos, jointPositions[i]);
               data.qvel().put(dof, 0.0);
            }
            simulated[i] = data.qpos().get(qpos);
         }
         return simulated;
      }

      private static String pose(RigidBodyTransformReadOnly transform)
      {
         Quaternion rotation = new Quaternion(transform.getRotation());
         return "pos=\"" + fmt(transform.getTranslation().getX()) + " " + fmt(transform.getTranslation().getY()) + " "
                + fmt(transform.getTranslation().getZ()) + "\" quat=\"" + fmt(rotation.getS()) + " " + fmt(rotation.getX()) + " "
                + fmt(rotation.getY()) + " " + fmt(rotation.getZ()) + "\"";
      }

      private static String fmt(double value)
      {
         return String.format(Locale.US, "%.8g", value);
      }
   }

   /** Model addresses of one placed object: its welded root body, then {qpos, dof} for each movable joint. */
   private record Binding(int rootBody, int[][] joints)
   {
   }

   // -------------------------------------------------------------------------------------------
   // Part definitions
   // -------------------------------------------------------------------------------------------

   private enum JointType
   {
      ROOT, FIXED, REVOLUTE, CONTINUOUS, PRISMATIC;

      boolean isMovable()
      {
         return this == REVOLUTE || this == CONTINUOUS || this == PRISMATIC;
      }

      static JointType parse(URDFJoint joint)
      {
         return switch (joint.getType())
         {
            case "fixed" -> FIXED;
            case "revolute" -> REVOLUTE;
            case "continuous" -> CONTINUOUS;
            case "prismatic" -> PRISMATIC;
            default -> throw new IllegalArgumentException("Joint " + joint.getName() + " is " + joint.getType()
                                                          + ". Supported types are fixed, revolute, continuous and prismatic.");
         };
      }
   }

   private record MeshReference(File file, RigidBodyTransform origin)
   {
   }

   private record PlacedMesh(RDXModelInstance instance, RigidBodyTransform origin)
   {
   }

   /** One URDF link and the joint that connects it to its parent. Never changes after loading. */
   private static final class PartDefinition
   {
      private final String name;
      private final int parentIndex;
      private final JointType type;
      private final String jointName;
      private final RigidBodyTransform jointOrigin;
      private final Vector3D axis;
      private final double lower;
      private final double upper;
      private final double damping;
      private final double friction;
      private final double mass;
      private final Matrix3D inertia;
      private final RigidBodyTransform inertiaPose;
      private final List<MeshReference> visuals;
      private final List<MeshReference> collisions;

      private PartDefinition(String name,
                             int parentIndex,
                             JointType type,
                             String jointName,
                             RigidBodyTransform jointOrigin,
                             Vector3D axis,
                             double lower,
                             double upper,
                             double damping,
                             double friction,
                             double mass,
                             Matrix3D inertia,
                             RigidBodyTransform inertiaPose,
                             List<MeshReference> visuals,
                             List<MeshReference> collisions)
      {
         this.name = name;
         this.parentIndex = parentIndex;
         this.type = type;
         this.jointName = jointName;
         this.jointOrigin = jointOrigin;
         this.axis = axis;
         this.lower = lower;
         this.upper = upper;
         this.damping = damping;
         this.friction = friction;
         this.mass = mass;
         this.inertia = inertia;
         this.inertiaPose = inertiaPose;
         this.visuals = visuals;
         this.collisions = collisions;
      }

      static PartDefinition from(URDFLink link, URDFJoint joint, int parentIndex, File directory)
      {
         URDFTools.URDFParserProperties properties = URDFTools.DEFAULT_URDF_PARSER_PROPERTIES;
         URDFInertial inertial = link.getInertial();
         double mass = URDFTools.parseMass(inertial == null ? null : inertial.getMass(), properties);
         Matrix3D inertia = URDFTools.parseMomentOfInertia(inertial == null ? null : inertial.getInertia(), properties);
         RigidBodyTransform inertiaPose = transform(inertial == null ? null : inertial.getOrigin());

         List<MeshReference> visuals = new ArrayList<>();
         for (URDFVisual visual : nonNull(link.getVisual()))
            addMesh(visuals, link.getName(), "visual", visual.getGeometry(), visual.getOrigin(), directory);
         List<MeshReference> collisions = new ArrayList<>();
         for (URDFCollision collision : nonNull(link.getCollision()))
            addMesh(collisions, link.getName(), "collision", collision.getGeometry(), collision.getOrigin(), directory);

         if (joint == null)
            return new PartDefinition(link.getName(), -1, JointType.ROOT, null, new RigidBodyTransform(), new Vector3D(),
                                      0.0, 0.0, 0.0, 0.0, mass, inertia, inertiaPose, visuals, collisions);

         JointType type = JointType.parse(joint);
         Vector3D axis = new Vector3D(URDFTools.parseAxis(joint.getAxis(), properties));
         if (type.isMovable() && axis.norm() < 1.0e-9)
            throw new IllegalArgumentException("Joint " + joint.getName() + " has a zero axis");
         if (type.isMovable())
            axis.normalize();
         URDFLimit limit = joint.getLimit();
         double lower = limit == null ? Double.NEGATIVE_INFINITY : URDFTools.parseDouble(limit.getLower(), Double.NEGATIVE_INFINITY);
         double upper = limit == null ? Double.POSITIVE_INFINITY : URDFTools.parseDouble(limit.getUpper(), Double.POSITIVE_INFINITY);
         if (type == JointType.CONTINUOUS)
         {
            lower = Double.NEGATIVE_INFINITY;
            upper = Double.POSITIVE_INFINITY;
         }
         URDFDynamics dynamics = joint.getDynamics();
         double damping = dynamics == null ? 0.0 : URDFTools.parseDouble(dynamics.getDamping(), 0.0);
         double friction = dynamics == null ? 0.0 : URDFTools.parseDouble(dynamics.getFriction(), 0.0);
         return new PartDefinition(link.getName(), parentIndex, type, joint.getName(), transform(joint.getOrigin()), axis,
                                   lower, upper, damping, friction, mass, inertia, inertiaPose, visuals, collisions);
      }

      private static void addMesh(List<MeshReference> meshes, String link, String kind, URDFGeometry geometry, URDFOrigin origin, File directory)
      {
         if (geometry == null || geometry.getMesh() == null)
         {
            LogTools.warn("Link {} has a {} that is not a mesh. Only meshes are supported, so it is skipped.", link, kind);
            return;
         }
         String scale = geometry.getMesh().getScale();
         if (scale != null && !scale.isBlank() && !scale.strip().matches("1(\\.0*)?\\s+1(\\.0*)?\\s+1(\\.0*)?"))
            LogTools.warn("Link {} {} mesh scale {} is ignored", link, kind, scale);
         String filename = geometry.getMesh().getFilename();
         if (filename == null)
            throw new IllegalArgumentException("Link " + link + " has a " + kind + " mesh whose file could not be resolved");
         File file = new File(filename);
         if (!file.isAbsolute())
            file = new File(directory, filename);
         if (!file.isFile())
            throw new IllegalArgumentException("Link " + link + " " + kind + " mesh not found: " + file);
         meshes.add(new MeshReference(file, transform(origin)));
      }

      private static RigidBodyTransform transform(URDFOrigin origin)
      {
         return new RigidBodyTransform(URDFTools.parseRigidBodyTransform(origin, URDFTools.DEFAULT_URDF_PARSER_PROPERTIES));
      }

      double clamp(double position)
      {
         if (!type.isMovable())
            return 0.0;
         double lowest = Double.isFinite(lower) ? lower : Double.NEGATIVE_INFINITY;
         double highest = Double.isFinite(upper) ? upper : Double.POSITIVE_INFINITY;
         return Math.min(highest, Math.max(lowest, position));
      }

      double sliderLower()
      {
         return Double.isFinite(lower) ? lower : -Math.PI;
      }

      double sliderUpper()
      {
         return Double.isFinite(upper) ? upper : Math.PI;
      }

      /** The joint's motion at {@code position}: a rotation about the axis, or a translation along it. */
      void motion(double position, AxisAngle axisAngle, RigidBodyTransform motionToPack)
      {
         motionToPack.setToZero();
         if (type == JointType.REVOLUTE || type == JointType.CONTINUOUS)
         {
            axisAngle.set((Vector3DReadOnly) axis, position);
            motionToPack.getRotation().set(axisAngle);
         }
         else if (type == JointType.PRISMATIC)
         {
            motionToPack.getTranslation().setAndScale(position, axis);
         }
      }
   }
}
