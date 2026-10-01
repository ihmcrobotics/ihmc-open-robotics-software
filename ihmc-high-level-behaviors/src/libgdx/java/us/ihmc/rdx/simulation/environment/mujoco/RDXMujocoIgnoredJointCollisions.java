package us.ihmc.rdx.simulation.environment.mujoco;

import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.definition.YawPitchRollTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Finger joints are listed in {@code nameOfJointsToIgnore}, so the MuJoCo exporter never creates
 * them. This writes those links the same way the exporter writes every other link: a body at
 * {@code transformToParent}, a hinge or slide when the joint has a degree of freedom, an inertial,
 * the collision geoms, and a parent-child contact exclude. The hand controller still owns the
 * commanded angle. That angle is the spring reference, not a hard qpos write, so the hinge can
 * stop when it meets an object. The pad contact itself stays stiff, so the visible finger rests
 * on the surface instead of sinking into it.
 */
public final class RDXMujocoIgnoredJointCollisions
{
   /** Slide, torsion, roll. The pads are a gum-like rubber, so tangential and torsional stiction are high. */
   private static final String FINGER_FRICTION = "2.5 1.0 0.05";
   /** Same contact time constant as the rest of the robot, so the pad does not sink in. */
   private static final String FINGER_SOLREF = "0.005 1.0";
   /** Sub-millimeter constraint width. A wide ramp is what let the mesh sit inside the object. */
   private static final String FINGER_SOLIMP = "0.9 0.99 0.0007 0.5 2.0";
   /** Nm/rad. A blocked finger yields around the object instead of holding the commanded angle. */
   private static final double FINGER_STIFFNESS = 0.25;
   /** Nm/(rad/s). The URDF damping is nearly zero, which rings once the hinge is no longer locked. */
   private static final double FINGER_DAMPING = 0.04;
   /** Extra joint inertia so a near-massless link does not explode under the spring. */
   private static final double FINGER_ARMATURE = 5.0e-4;

   private static final Pattern MESH_ASSET = Pattern.compile("<mesh name=\"([^\"]+)\"");
   private static final Pattern BODY_NAME = Pattern.compile("<body name=\"([^\"]+)\"");

   private final List<Hand> hands;
   private final List<DrivenJoint> drivenJoints;
   private int emittedDofs;
   private final Quaternion quaternion = new Quaternion();
   private boolean configurationErrorLogged;
   private boolean springsSeeded;

   private RDXMujocoIgnoredJointCollisions(List<Hand> hands, List<DrivenJoint> drivenJoints)
   {
      this.hands = hands;
      this.drivenJoints = drivenJoints;
      this.emittedDofs = drivenJoints.size();
   }

   public static RDXMujocoIgnoredJointCollisions none()
   {
      return new RDXMujocoIgnoredJointCollisions(List.of(), List.of());
   }

   public static RDXMujocoIgnoredJointCollisions fromRobot(Robot robot)
   {
      RobotDefinition robotDefinition = robot.getRobotDefinition();
      Set<String> ignored = new HashSet<>(robotDefinition.getNameOfJointsToIgnore());
      String prefix = robotDefinition.getName() + "_";
      Set<String> emittedBodies = new HashSet<>();
      List<Hand> hands = new ArrayList<>();
      List<DrivenJoint> drivenJoints = new ArrayList<>();
      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         if (!ignored.contains(joint.getName()) || joint.getSuccessor() == null)
            continue;
         JointDefinition parent = joint.getParentJoint();
         if (parent != null && ignored.contains(parent.getName()))
            continue;
         if (parent == null || parent.getSuccessor() == null)
            continue;
         if (robot.getRigidBody(joint.getSuccessor().getName()) == null)
            continue;

         ChainLink root = build(joint, prefix, emittedBodies, drivenJoints);
         if (root == null)
            continue;
         String wristBodyName = null;
         if (parent.getParentJoint() != null && parent.getParentJoint().getSuccessor() != null)
            wristBodyName = prefix + parent.getParentJoint().getSuccessor().getName();
         hands.add(new Hand(prefix + parent.getSuccessor().getName(), wristBodyName, root));
      }
      if (!hands.isEmpty())
         LogTools.info("MuJoCo will collide {} ignored robot link(s) as regular links, including the fingers", emittedBodies.size());
      return new RDXMujocoIgnoredJointCollisions(hands, List.copyOf(drivenJoints));
   }

   public boolean isEmpty()
   {
      return hands.isEmpty();
   }

   /** Revolute and prismatic joints added under the grippers. Each one adds one {@code nq} and one {@code nv}. */
   public int addedDofCount()
   {
      return emittedDofs;
   }

   /**
    * Inserts the ignored subtrees inside the simulated body each one hangs off, and adds the same
    * parent-child contact excludes the exporter writes for the rest of the robot.
    */
   public String injectInto(String xml, RobotDefinition robotDefinition)
   {
      if (hands.isEmpty())
         return xml;

      emittedDofs = 0;
      Set<String> meshAssets = new HashSet<>();
      Matcher meshNames = MESH_ASSET.matcher(xml);
      while (meshNames.find())
         meshAssets.add(meshNames.group(1));
      Set<String> existingBodies = new HashSet<>();
      Matcher bodyNames = BODY_NAME.matcher(xml);
      while (bodyNames.find())
         existingBodies.add(bodyNames.group(1));

      String result = xml;
      StringBuilder excludes = new StringBuilder();
      for (Hand hand : hands)
      {
         StringBuilder bodies = new StringBuilder();
         appendLink(bodies, excludes, hand.root, hand.anchorBodyName, hand.anchorBodyName, hand.wristBodyName, meshAssets, existingBodies);
         int closing = closingBodyIndex(result, hand.anchorBodyName);
         if (closing < 0)
            throw new IllegalStateException("MuJoCo body " + hand.anchorBodyName + " is missing; finger links were not added");
         result = result.substring(0, closing) + bodies + result.substring(closing);
      }

      if (excludes.isEmpty())
         return result;
      int contactEnd = result.lastIndexOf("</contact>");
      if (contactEnd >= 0)
         return result.substring(0, contactEnd) + excludes + result.substring(contactEnd);
      int mujocoEnd = result.lastIndexOf("</mujoco>");
      if (mujocoEnd < 0)
         throw new IllegalStateException("MuJoCo world XML has no </mujoco>");
      return result.substring(0, mujocoEnd) + "  <contact>\n" + excludes + "  </contact>\n" + result.substring(mujocoEnd);
   }

   /**
    * Points each finger spring at the hand controller's angle. Contact is free to deflect the hinge.
    * The deflected angle is copied into the controller output so the visible hand matches MuJoCo.
    */
   public void writeConfiguration(Robot robot, mjModel model, mjData data)
   {
      if (drivenJoints.isEmpty() || robot == null || model == null || model.isNull() || data == null || data.isNull())
         return;
      try
      {
         ControllerOutput output = robot.getControllerManager().getControllerOutput();
         boolean wrote = false;
         for (DrivenJoint driven : drivenJoints)
         {
            SimJointBasics joint = robot.getJoint(driven.scs2Name);
            if (!(joint instanceof OneDoFJointBasics oneDof))
               continue;
            int jointId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_JOINT, driven.mujocoName);
            if (jointId < 0)
               continue;
            int qadr = model.jnt_qposadr().get(jointId);
            int vadr = model.jnt_dofadr().get(jointId);
            double commanded = oneDof.getQ();
            model.qpos_spring().put(qadr, commanded);
            if (!springsSeeded)
            {
               data.qpos().put(qadr, commanded);
               data.qvel().put(vadr, 0.0);
            }
            wrote = true;
            if (springsSeeded)
               copyDeflectionToHand(output, oneDof, data.qpos().get(qadr), data.qvel().get(vadr));
         }
         if (wrote)
            springsSeeded = true;
      }
      catch (RuntimeException exception)
      {
         if (!configurationErrorLogged)
         {
            configurationErrorLogged = true;
            LogTools.error("Finger joint configuration was not copied into MuJoCo: {}", exception.getMessage());
         }
      }
   }

   private static void copyDeflectionToHand(ControllerOutput output, OneDoFJointBasics joint, double angle, double velocity)
   {
      if (output == null)
         return;
      OneDoFJointStateBasics jointOutput = output.getOneDoFJointOutput(joint);
      if (jointOutput == null)
         return;
      jointOutput.setConfiguration(angle);
      jointOutput.setVelocity(velocity);
   }

   private static ChainLink build(JointDefinition joint, String prefix, Set<String> emittedBodies, List<DrivenJoint> drivenJoints)
   {
      RigidBodyDefinition body = joint.getSuccessor();
      if (body == null || !emittedBodies.add(body.getName()))
         return null;

      List<ChainLink> children = new ArrayList<>();
      for (JointDefinition child : body.getChildrenJoints())
      {
         ChainLink childLink = build(child, prefix, emittedBodies, drivenJoints);
         if (childLink != null)
            children.add(childLink);
      }

      boolean oneDof = joint instanceof RevoluteJointDefinition || joint instanceof PrismaticJointDefinition;
      if (!oneDof && body.getCollisionShapeDefinitions().isEmpty() && children.isEmpty())
      {
         emittedBodies.remove(body.getName());
         return null;
      }
      String mujocoJointName = prefix + joint.getName();
      if (oneDof)
         drivenJoints.add(new DrivenJoint(joint.getName(), mujocoJointName));
      return new ChainLink(joint, body, prefix + body.getName(), mujocoJointName, children);
   }

   private void appendLink(StringBuilder bodies,
                           StringBuilder excludes,
                           ChainLink link,
                           String parentBodyName,
                           String anchorBodyName,
                           String wristBodyName,
                           Set<String> meshAssets,
                           Set<String> existingBodies)
   {
      // A loop joint can name a body the exporter already wrote. A second copy would fail to compile.
      if (existingBodies.contains(link.bodyName))
         return;
      String pad = "        ";
      bodies.append(pad).append("<body name=\"").append(link.bodyName).append('"');
      YawPitchRollTransformDefinition transform = link.joint.getTransformToParent();
      if (transform.hasTranslation() || transform.hasRotation())
      {
         quaternion.set(transform.getRotation());
         bodies.append(" pos=\"").append(fmt(transform.getTranslation().getX())).append(' ')
                .append(fmt(transform.getTranslation().getY())).append(' ')
                .append(fmt(transform.getTranslation().getZ())).append("\" quat=\"")
                .append(fmt(quaternion.getS())).append(' ')
                .append(fmt(quaternion.getX())).append(' ')
                .append(fmt(quaternion.getY())).append(' ')
                .append(fmt(quaternion.getZ())).append('"');
      }
      bodies.append(">\n");
      appendJoint(bodies, link);
      appendInertial(bodies, link.body);

      int geomIndex = 0;
      for (CollisionShapeDefinition shape : link.body.getCollisionShapeDefinitions())
      {
         String geom = geomXml(link.bodyName + "_geom_" + geomIndex, shape, meshAssets);
         if (geom != null)
            bodies.append(geom);
         geomIndex++;
      }
      for (ChainLink child : link.children)
         appendLink(bodies, excludes, child, link.bodyName, anchorBodyName, wristBodyName, meshAssets, existingBodies);
      bodies.append(pad).append("</body>\n");

      appendExclude(excludes, parentBodyName, link.bodyName);
      if (!anchorBodyName.equals(parentBodyName))
         appendExclude(excludes, anchorBodyName, link.bodyName);
      if (wristBodyName != null)
         appendExclude(excludes, wristBodyName, link.bodyName);
   }

   private void appendJoint(StringBuilder bodies, ChainLink link)
   {
      if (!(link.joint instanceof OneDoFJointDefinition oneDof))
         return;
      if (!(link.joint instanceof RevoluteJointDefinition) && !(link.joint instanceof PrismaticJointDefinition))
         return;
      String type = link.joint instanceof RevoluteJointDefinition ? "hinge" : "slide";
      bodies.append("          <joint name=\"").append(link.mujocoJointName).append("\" type=\"").append(type)
             .append("\" axis=\"").append(fmt(oneDof.getAxis().getX())).append(' ')
             .append(fmt(oneDof.getAxis().getY())).append(' ')
             .append(fmt(oneDof.getAxis().getZ())).append('"')
             .append(" damping=\"").append(fmt(FINGER_DAMPING)).append('"')
             .append(" stiffness=\"").append(fmt(FINGER_STIFFNESS)).append('"')
             .append(" armature=\"").append(fmt(FINGER_ARMATURE)).append('"');
      double lower = oneDof.getPositionLowerLimit();
      double upper = oneDof.getPositionUpperLimit();
      if (Double.isFinite(lower) && Double.isFinite(upper) && upper > lower && upper - lower < 7.0)
         bodies.append(" limited=\"true\" range=\"").append(fmt(lower)).append(' ').append(fmt(upper)).append('"');
      bodies.append("/>\n");
      emittedDofs++;
   }

   private void appendInertial(StringBuilder bodies, RigidBodyDefinition body)
   {
      var inertia = body.getMomentOfInertia();
      bodies.append("          <inertial pos=\"").append(fmt(body.getCenterOfMassOffset().getX())).append(' ')
             .append(fmt(body.getCenterOfMassOffset().getY())).append(' ')
             .append(fmt(body.getCenterOfMassOffset().getZ())).append('"')
             .append(" mass=\"").append(fmt(body.getMass())).append('"');
      if (body.getMass() > 0.0)
      {
         bodies.append(" fullinertia=\"")
                .append(fmt(inertia.getM00())).append(' ')
                .append(fmt(inertia.getM11())).append(' ')
                .append(fmt(inertia.getM22())).append(' ')
                .append(fmt(inertia.getM01())).append(' ')
                .append(fmt(inertia.getM02())).append(' ')
                .append(fmt(inertia.getM12())).append('"');
      }
      bodies.append("/>\n");
   }

   private static void appendExclude(StringBuilder excludes, String body1, String body2)
   {
      excludes.append("      <exclude body1=\"").append(body1).append("\" body2=\"").append(body2).append("\"/>\n");
   }

   /** Index of the {@code </body>} that closes the named body, accounting for nested bodies. */
   private static int closingBodyIndex(String xml, String bodyName)
   {
      int start = xml.indexOf("<body name=\"" + bodyName + "\"");
      if (start < 0)
         return -1;
      int depth = 0;
      int at = start;
      while (at < xml.length())
      {
         int open = xml.indexOf("<body", at);
         int close = xml.indexOf("</body>", at);
         if (close < 0)
            return -1;
         if (open >= 0 && open < close)
         {
            depth++;
            at = open + "<body".length();
         }
         else
         {
            depth--;
            if (depth == 0)
               return close;
            at = close + "</body>".length();
         }
      }
      return -1;
   }

   private String geomXml(String geomName, CollisionShapeDefinition shape, Set<String> meshAssets)
   {
      GeometryDefinition geometry = shape.getGeometryDefinition();
      String size;
      if (geometry instanceof Box3DDefinition box)
      {
         size = "type=\"box\" size=\"" + fmt(box.getSizeX() / 2.0) + " " + fmt(box.getSizeY() / 2.0) + " " + fmt(box.getSizeZ() / 2.0) + "\"";
      }
      else if (geometry instanceof Sphere3DDefinition sphere)
      {
         size = "type=\"sphere\" size=\"" + fmt(sphere.getRadius()) + "\"";
      }
      else if (geometry instanceof Cylinder3DDefinition cylinder)
      {
         size = "type=\"cylinder\" size=\"" + fmt(cylinder.getRadius()) + " " + fmt(cylinder.getLength() / 2.0) + "\"";
      }
      else if (geometry instanceof Capsule3DDefinition capsule)
      {
         size = "type=\"capsule\" size=\"" + fmt(capsule.getRadiusX()) + " " + fmt(capsule.getLength() / 2.0) + "\"";
      }
      else if (geometry instanceof ModelFileGeometryDefinition modelFile)
      {
         String fileName = modelFile.getFileName();
         if (fileName == null)
            return null;
         String lower = fileName.toLowerCase(Locale.US);
         if (!lower.endsWith(".stl") && !lower.endsWith(".obj"))
            return null;
         String meshName = geomName + "_mesh";
         if (!meshAssets.contains(meshName))
            return null;
         size = "type=\"mesh\" mesh=\"" + meshName + "\"";
      }
      else
      {
         return null;
      }

      String pose = "";
      var origin = shape.getOriginPose();
      if (origin.hasTranslation() || origin.hasRotation())
      {
         quaternion.set(origin.getRotation());
         pose = " pos=\"" + fmt(origin.getTranslation().getX()) + " "
                + fmt(origin.getTranslation().getY()) + " "
                + fmt(origin.getTranslation().getZ()) + "\" quat=\""
                + fmt(quaternion.getS()) + " " + fmt(quaternion.getX()) + " "
                + fmt(quaternion.getY()) + " " + fmt(quaternion.getZ()) + "\"";
      }
      return "          <geom class=\"robot\" name=\"" + geomName + "\"" + pose
             + " friction=\"" + FINGER_FRICTION + "\" condim=\"6\" solref=\"" + FINGER_SOLREF
             + "\" solimp=\"" + FINGER_SOLIMP + "\" " + size + "/>\n";
   }

   private static String fmt(double value)
   {
      return String.format(Locale.US, "%.8g", value);
   }

   private static final class Hand
   {
      private final String anchorBodyName;
      private final String wristBodyName;
      private final ChainLink root;

      private Hand(String anchorBodyName, String wristBodyName, ChainLink root)
      {
         this.anchorBodyName = anchorBodyName;
         this.wristBodyName = wristBodyName;
         this.root = root;
      }
   }

   private static final class ChainLink
   {
      private final JointDefinition joint;
      private final RigidBodyDefinition body;
      private final String bodyName;
      private final String mujocoJointName;
      private final List<ChainLink> children;

      private ChainLink(JointDefinition joint, RigidBodyDefinition body, String bodyName, String mujocoJointName, List<ChainLink> children)
      {
         this.joint = joint;
         this.body = body;
         this.bodyName = bodyName;
         this.mujocoJointName = mujocoJointName;
         this.children = children;
      }
   }

   private static final class DrivenJoint
   {
      private final String scs2Name;
      private final String mujocoName;

      private DrivenJoint(String scs2Name, String mujocoName)
      {
         this.scs2Name = scs2Name;
         this.mujocoName = mujocoName;
      }
   }
}
