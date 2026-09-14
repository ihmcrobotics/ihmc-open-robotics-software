package us.ihmc.scs2.simulation.mujoco.physicsEngine;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.DefinitionIOTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoCone;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoIntegrator;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoJacobian;
import us.ihmc.scs2.simulation.mujoco.physicsEngine.parameters.MujocoSolver;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;

/**
 * Pure-Java MJCF generation helpers used by the MuJoCo SCS2 integration. No JNI in this class.
 */
public final class MujocoTools
{
   private MujocoTools()
   {
   }

   /**
    * MuJoCo XML uses {@code pos="x y z"} and {@code quat="w x y z"} (w-first). Converts a
    * {@link RigidBodyTransformReadOnly} into the MJCF attribute fragment
    * {@code "pos=\"x y z\" quat=\"w x y z\""}.
    */
   public static String toPosQuatAttributes(RigidBodyTransformReadOnly transform)
   {
      return toPosQuatAttributes(transform.getTranslation(), transform.getRotation());
   }

   /**
    * Same as {@link #toPosQuatAttributes(RigidBodyTransformReadOnly)} but takes the translation
    * and orientation separately. Accepts any {@link Orientation3DReadOnly} (quaternion, rotation
    * matrix, or axis-angle); a temporary {@link Quaternion} is used to pull out the (w, x, y, z)
    * components in the MuJoCo order.
    */
   public static String toPosQuatAttributes(Tuple3DReadOnly translation, Orientation3DReadOnly rotation)
   {
      Quaternion q = new Quaternion();
      q.set(rotation);
      return new StringBuilder()
         .append("pos=\"")
         .append(translation.getX()).append(' ')
         .append(translation.getY()).append(' ')
         .append(translation.getZ()).append("\" quat=\"")
         .append(q.getS()).append(' ')
         .append(q.getX()).append(' ')
         .append(q.getY()).append(' ')
         .append(q.getZ()).append('"')
         .toString();
   }

   /**
    * Emit a {@code <geom>} for one collision shape under the given collision class ("robot" or
    * "terrain"). Supports box / sphere / cylinder / capsule / ellipsoid directly. Convex polytopes and ramps are
    * emitted as {@code type="mesh"} referencing a {@code <mesh>} that {@link #appendMeshAsset} must
    * have declared under the same {@code name} (MuJoCo builds the convex hull from the vertex cloud).
    * Any other geometry emits a TODO comment instead.
    */
   static void appendGeom(StringBuilder sb, String geomClass, String name, CollisionShapeDefinition shape, int indent)
   {
      String pad = "  ".repeat(indent);
      GeometryDefinition geometry = shape.getGeometryDefinition();
      sb.append(pad).append("<geom class=\"").append(geomClass).append("\" name=\"").append(name).append('"');
      if (!isIdentity(shape.getOriginPose()))
         sb.append(' ').append(toPosQuatAttributes(shape.getOriginPose()));

      if (geometry instanceof Box3DDefinition box)
      {
         // MuJoCo box size is half-extents.
         sb.append(" type=\"box\" size=\"")
           .append(box.getSizeX() / 2.0).append(' ')
           .append(box.getSizeY() / 2.0).append(' ')
           .append(box.getSizeZ() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Sphere3DDefinition sphere)
      {
         sb.append(" type=\"sphere\" size=\"").append(sphere.getRadius()).append("\"/>\n");
      }
      else if (geometry instanceof Cylinder3DDefinition cylinder)
      {
         // MuJoCo cylinder size: (radius, half-length).
         sb.append(" type=\"cylinder\" size=\"")
           .append(cylinder.getRadius()).append(' ')
           .append(cylinder.getLength() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Capsule3DDefinition capsule)
      {
         // MuJoCo capsule size: (radius, half-length-of-cylinder-section).
         sb.append(" type=\"capsule\" size=\"")
           .append(capsule.getRadiusX()).append(' ')
           .append(capsule.getLength() / 2.0).append("\"/>\n");
      }
      else if (geometry instanceof Ellipsoid3DDefinition ellipsoid)
      {
         // MuJoCo ellipsoid size is the three semi-axis radii.
         sb.append(" type=\"ellipsoid\" size=\"")
           .append(ellipsoid.getRadiusX()).append(' ')
           .append(ellipsoid.getRadiusY()).append(' ')
           .append(ellipsoid.getRadiusZ()).append("\"/>\n");
      }
      else if (geometry instanceof ModelFileGeometryDefinition modelFile)
      {
         if (isSupportedModelFileMesh(modelFile))
         {
            // Mesh assets carry no size; the geom just references the <mesh> declared by appendMeshAsset.
            sb.append(" type=\"mesh\" mesh=\"").append(meshName(name)).append("\"/>\n");
         }
         else
         {
            sb.append("/><!-- skipped unsupported model file: ").append(modelFile.getFileName()).append(" -->\n");
         }
      }
      else if (isMeshGeometry(geometry))
      {
         // Mesh assets carry no size; the geom just references the <mesh> declared by appendMeshAsset.
         sb.append(" type=\"mesh\" mesh=\"").append(meshName(name)).append("\"/>\n");
      }
      else
      {
         sb.append("/><!-- TODO unsupported geometry: ").append(geometry.getClass().getSimpleName()).append(" -->\n");
      }
   }

   /** True if the geometry is emitted as a MuJoCo mesh (polytope, ramp, or supported model file). */
   static boolean isMeshGeometry(GeometryDefinition geometry)
   {
      if (geometry instanceof ConvexPolytope3DDefinition || geometry instanceof Ramp3DDefinition)
         return true;
      return geometry instanceof ModelFileGeometryDefinition modelFile && isSupportedModelFileMesh(modelFile);
   }

   static boolean isSupportedModelFileMesh(ModelFileGeometryDefinition modelFile)
   {
      String fileName = modelFile.getFileName();
      if (fileName == null)
         return false;
      String lower = fileName.toLowerCase();
      return lower.endsWith(".stl") || lower.endsWith(".obj");
   }

   /** The {@code <mesh>} asset name paired with the {@code <geom>} of the same {@code name}. */
   static String meshName(String geomName)
   {
      return geomName + "_mesh";
   }

   /**
    * If {@code shape} is a mesh geometry, emit its {@code <mesh>} asset. Convex polytopes / ramps use an
    * inline vertex cloud (MuJoCo builds the convex hull). {@link ModelFileGeometryDefinition} meshes
    * (STL/OBJ) are copied into {@code workingDirectory} and referenced via {@code file="..."}.
    * No-op for primitives. Pass {@code workingDirectory == null} only when no model-file meshes are present.
    */
   static void appendMeshAsset(StringBuilder sb, String name, CollisionShapeDefinition shape, int indent, File workingDirectory)
   {
      GeometryDefinition geometry = shape.getGeometryDefinition();
      if (!isMeshGeometry(geometry))
         return;

      if (geometry instanceof ModelFileGeometryDefinition modelFile)
      {
         appendModelFileMeshAsset(sb, name, modelFile, indent, workingDirectory);
         return;
      }

      sb.append("  ".repeat(indent)).append("<mesh name=\"").append(meshName(name)).append("\" vertex=\"");
      if (geometry instanceof ConvexPolytope3DDefinition polytope)
      {
         for (Vertex3DReadOnly vertex : polytope.getConvexPolytope().getVertices())
            appendVertex(sb, vertex.getX(), vertex.getY(), vertex.getZ());
      }
      else if (geometry instanceof Ramp3DDefinition ramp)
      {
         appendRampVertices(sb, ramp);
      }
      sb.append("\"/>\n");
   }

   /** Backward-compatible overload used by terrain (no model-file meshes). */
   static void appendMeshAsset(StringBuilder sb, String name, CollisionShapeDefinition shape, int indent)
   {
      appendMeshAsset(sb, name, shape, indent, null);
   }

   private static void appendModelFileMeshAsset(StringBuilder sb,
                                                String name,
                                                ModelFileGeometryDefinition modelFile,
                                                int indent,
                                                File workingDirectory)
   {
      if (workingDirectory == null)
      {
         LogTools.warn("Skipping MuJoCo mesh asset '{}': working directory is null", modelFile.getFileName());
         return;
      }

      String sourceFileName = modelFile.getFileName();
      if (!isSupportedModelFileMesh(modelFile))
      {
         LogTools.warn("Skipping MuJoCo mesh asset '{}': only .stl/.obj are supported", sourceFileName);
         return;
      }

      try
      {
         URL sourceURL = DefinitionIOTools.resolveModelFileURL(modelFile);
         String lower = sourceFileName.toLowerCase();
         String extension = lower.substring(lower.lastIndexOf('.'));
         File meshFile = new File(workingDirectory, meshName(name) + extension);
         try (InputStream in = sourceURL.openStream())
         {
            Files.copy(in, meshFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
         }

         sb.append("  ".repeat(indent)).append("<mesh name=\"").append(meshName(name))
           .append("\" file=\"").append(meshFile.getName()).append('"');
         Vector3DReadOnly scale = modelFile.getScale();
         if (scale != null && (scale.getX() != 1.0 || scale.getY() != 1.0 || scale.getZ() != 1.0))
         {
            sb.append(" scale=\"").append(scale.getX()).append(' ').append(scale.getY()).append(' ').append(scale.getZ()).append('"');
         }
         sb.append("/>\n");
      }
      catch (IOException | RuntimeException e)
      {
         throw new RuntimeException("Failed to stage MuJoCo mesh '" + sourceFileName + "' into " + workingDirectory, e);
      }
   }

   /**
    * Append the 6 corner vertices of a ramp (a triangular prism) as a flat vertex list. Matches the
    * SCS2 / Bullet convention: the bottom face is centered at the origin and the slope rises toward
    * +x, reaching {@code sizeZ} at the +x end.
    */
   private static void appendRampVertices(StringBuilder sb, Ramp3DDefinition ramp)
   {
      double x = 0.5 * ramp.getSizeX();
      double y = 0.5 * ramp.getSizeY();
      double z = ramp.getSizeZ();
      // bottom rectangle (z=0) then the two high-edge corners at the +x end (z=sizeZ).
      appendVertex(sb, -x, -y, 0.0);
      appendVertex(sb, -x, y, 0.0);
      appendVertex(sb, x, y, 0.0);
      appendVertex(sb, x, -y, 0.0);
      appendVertex(sb, x, y, z);
      appendVertex(sb, x, -y, z);
   }

   private static void appendVertex(StringBuilder sb, double x, double y, double z)
   {
      if (sb.charAt(sb.length() - 1) != '"')
         sb.append("  ");
      sb.append(x).append(' ').append(y).append(' ').append(z);
   }

   /** Emit a body's {@code <inertial>} element (CoM offset, mass, and full inertia tensor). */
   static void appendInertial(StringBuilder sb, RigidBodyDefinition body, int indent)
   {
      String pad = "  ".repeat(indent);
      Tuple3DReadOnly comOffset = body.getCenterOfMassOffset();
      var inertia = body.getMomentOfInertia();

      sb.append(pad).append("<inertial")
        .append(" pos=\"").append(comOffset.getX()).append(' ').append(comOffset.getY()).append(' ').append(comOffset.getZ()).append('"')
        .append(" mass=\"").append(body.getMass()).append('"');
      if (body.getMass() > 0.0)
      {
         // fullinertia order: Ixx Iyy Izz Ixy Ixz Iyz
         sb.append(" fullinertia=\"")
           .append(inertia.getM00()).append(' ')
           .append(inertia.getM11()).append(' ')
           .append(inertia.getM22()).append(' ')
           .append(inertia.getM01()).append(' ')
           .append(inertia.getM02()).append(' ')
           .append(inertia.getM12())
           .append('"');
      }
      sb.append("/>\n");
   }

   /** Emit a joint's MJCF element: {@code <freejoint>} for the root, {@code <joint>} for 1-DoF. */
   static void appendJoint(StringBuilder sb, JointDefinition joint, String namePrefix, int indent)
   {
      String pad = "  ".repeat(indent);
      if (joint instanceof SixDoFJointDefinition)
      {
         sb.append(pad).append("<freejoint name=\"").append(namePrefix).append(joint.getName()).append("\"/>\n");
      }
      else if (joint instanceof RevoluteJointDefinition rev)
      {
         appendOneDofJoint(sb, pad, "hinge", namePrefix, rev);
      }
      else if (joint instanceof PrismaticJointDefinition pris)
      {
         appendOneDofJoint(sb, pad, "slide", namePrefix, pris);
      }
      else
      {
         sb.append(pad).append("<!-- TODO unsupported joint type: ").append(joint.getClass().getSimpleName())
           .append(" (name=").append(joint.getName()).append(") -->\n");
      }
   }

   private static void appendOneDofJoint(StringBuilder sb, String pad, String mjcfType, String namePrefix, OneDoFJointDefinition jointDef)
   {
      Tuple3DReadOnly axis = jointDef.getAxis();
      sb.append(pad).append("<joint name=\"").append(namePrefix).append(jointDef.getName())
        .append("\" type=\"").append(mjcfType)
        .append("\" axis=\"").append(axis.getX()).append(' ').append(axis.getY()).append(' ').append(axis.getZ()).append('"');

      // Damping
      if (jointDef.getDamping() > 0.0)
         sb.append(" damping=\"").append(jointDef.getDamping()).append('"');
      // PATCH (workspace shadow of scs2-mujoco-simulation 17-0.33.5): emit joint position limits.
      // The released factory emits no range, so every joint is physically unlimited in MuJoCo —
      // dynamics push joints past their mechanical stops (observed: ankle at -1.40 rad vs the
      // -1.047 URDF stop during RL walking, feet visually folding into the floor, forward-pitch
      // ratchet to a fall). RL policies trained against Isaac's URDF limits lean on hard stops
      // (toe-off), so the stops must exist here too. Uses MuJoCo's default (stiff) limit solver.
      double lower = jointDef.getPositionLowerLimit();
      double upper = jointDef.getPositionUpperLimit();
      if (Double.isFinite(lower) && Double.isFinite(upper) && lower < upper)
         sb.append(" limited=\"true\" range=\"").append(lower).append(' ').append(upper).append('"');
      sb.append("/>\n");
   }

   /** True if the transform has neither rotation nor translation (so pos/quat can be omitted). */
   static boolean isIdentity(RigidBodyTransformReadOnly transform)
   {
      return !transform.hasRotation() && !transform.hasTranslation();
   }

   private static final int MODEL_SUMMARY_MAX_GEOMS = 8;

   /** Logs the compiled model's effective option and per-geom contact values — what MuJoCo actually runs vs what was requested. */
   public static void logEffectiveModelSummary(Mujoco.mjModel model)
   {
      Mujoco.mjOption opt = model.opt();
      StringBuilder summary = new StringBuilder("MuJoCo compiled model summary:\n");
      summary.append(String.format("  option: timestep=%s integrator=%s solver=%s cone=%s jacobian=%s iterations=%d ls_iterations=%d tolerance=%s"
                                   + " impratio=%s noslip_iterations=%d%n",
                                   opt.timestep(),
                                   MujocoIntegrator.fromMujocoValue(opt.integrator()),
                                   MujocoSolver.fromMujocoValue(opt.solver()),
                                   MujocoCone.fromMujocoValue(opt.cone()),
                                   MujocoJacobian.fromMujocoValue(opt.jacobian()),
                                   opt.iterations(),
                                   opt.ls_iterations(),
                                   opt.tolerance(),
                                   opt.impratio(),
                                   opt.noslip_iterations()));

      int ngeom = (int) model.ngeom();
      int printed = Math.min(ngeom, MODEL_SUMMARY_MAX_GEOMS);
      DoublePointer friction = model.geom_friction();
      DoublePointer solref = model.geom_solref();
      DoublePointer solimp = model.geom_solimp();
      DoublePointer margin = model.geom_margin();
      DoublePointer gap = model.geom_gap();
      IntPointer condim = model.geom_condim();
      for (int geomId = 0; geomId < printed; geomId++)
      {
         BytePointer namePointer = Mujoco.mj_id2name(model, Mujoco.mjOBJ_GEOM, geomId);
         String name = namePointer == null || namePointer.isNull() ? "(unnamed)" : namePointer.getString();
         summary.append(String.format("  geom[%d] %s: friction=(%s %s %s) solref=(%s %s) solimp=(%s %s %s %s %s) condim=%d margin=%s gap=%s%n",
                                      geomId,
                                      name,
                                      friction.get(3 * geomId),
                                      friction.get(3 * geomId + 1),
                                      friction.get(3 * geomId + 2),
                                      solref.get(2L * geomId),
                                      solref.get(2L * geomId + 1),
                                      solimp.get(5L * geomId),
                                      solimp.get(5L * geomId + 1),
                                      solimp.get(5L * geomId + 2),
                                      solimp.get(5L * geomId + 3),
                                      solimp.get(5L * geomId + 4),
                                      condim.get(geomId),
                                      margin.get(geomId),
                                      gap.get(geomId)));
      }
      if (ngeom > printed)
         summary.append("  ... ").append(ngeom - printed).append(" more geoms elided\n");
      LogTools.info(summary.toString());
   }
}
