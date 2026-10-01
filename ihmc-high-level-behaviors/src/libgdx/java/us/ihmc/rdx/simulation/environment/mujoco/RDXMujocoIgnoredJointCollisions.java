package us.ihmc.rdx.simulation.environment.mujoco;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.mujoco.Mujoco;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjData;
import us.ihmc.scs2.simulation.mujoco.Mujoco.mjModel;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Finger links are on {@code nameOfJointsToIgnore}, so the MuJoCo exporter never creates their
 * bodies. Their mesh assets are still written. This puts each ignored link that has a collision
 * back in as a mocap body and drives it from the simulated link frame, which the hand controller
 * already poses.
 */
public final class RDXMujocoIgnoredJointCollisions
{
   private final List<Link> links;
   private final Set<String> anchorLinkNames;
   private final FramePose3D linkPose = new FramePose3D();
   private final Quaternion quaternion = new Quaternion();
   private final RigidBodyTransform origin = new RigidBodyTransform();
   private boolean poseErrorLogged;

   private RDXMujocoIgnoredJointCollisions(List<Link> links, Set<String> anchorLinkNames)
   {
      this.links = links;
      this.anchorLinkNames = anchorLinkNames;
   }

   public static RDXMujocoIgnoredJointCollisions none()
   {
      return new RDXMujocoIgnoredJointCollisions(List.of(), Set.of());
   }

   public static RDXMujocoIgnoredJointCollisions fromRobot(Robot robot)
   {
      RobotDefinition robotDefinition = robot.getRobotDefinition();
      Set<String> ignored = new HashSet<>(robotDefinition.getNameOfJointsToIgnore());
      String prefix = robotDefinition.getName() + "_";
      List<Link> links = new ArrayList<>();
      Set<String> anchorLinkNames = new HashSet<>();
      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         if (!ignored.contains(joint.getName()) || joint.getSuccessor() == null)
            continue;
         RigidBodyDefinition body = joint.getSuccessor();
         if (body.getCollisionShapeDefinitions().isEmpty())
            continue;
         if (robot.getRigidBody(body.getName()) == null)
            continue;

         Set<String> excludeBodies = new HashSet<>();
         JointDefinition ancestor = joint.getParentJoint();
         while (ancestor != null && ignored.contains(ancestor.getName()))
            ancestor = ancestor.getParentJoint();
         if (ancestor != null && ancestor.getSuccessor() != null)
         {
            anchorLinkNames.add(ancestor.getSuccessor().getName());
            excludeBodies.add(prefix + ancestor.getSuccessor().getName());
            JointDefinition wrist = ancestor.getParentJoint();
            if (wrist != null && wrist.getSuccessor() != null)
               excludeBodies.add(prefix + wrist.getSuccessor().getName());
         }
         links.add(new Link(body.getName(), prefix + body.getName(), excludeBodies));
      }
      if (!links.isEmpty())
         LogTools.info("MuJoCo will collide {} ignored robot link(s), including the fingers", links.size());
      return new RDXMujocoIgnoredJointCollisions(links, anchorLinkNames);
   }

   public boolean isEmpty()
   {
      return links.isEmpty();
   }

   public String bodyXml(RobotDefinition robotDefinition)
   {
      if (links.isEmpty())
         return "";
      String prefix = robotDefinition.getName() + "_";
      StringBuilder xml = new StringBuilder();
      xml.append("    <!-- ignored-joint-collisions -->\n");
      for (Link link : links)
      {
         RigidBodyDefinition body = robotDefinition.getRigidBodyDefinition(link.linkName);
         xml.append("    <body name=\"").append(link.bodyName).append("\" mocap=\"true\">\n");
         int geomIndex = 0;
         for (CollisionShapeDefinition shape : body.getCollisionShapeDefinitions())
         {
            String geom = geomXml(prefix + body.getName() + "_geom_" + geomIndex, shape);
            if (geom != null)
               xml.append(geom);
            geomIndex++;
         }
         xml.append("    </body>\n");
      }
      xml.append("    <!-- /ignored-joint-collisions -->\n");
      return xml.toString();
   }

   public String excludeXml()
   {
      if (links.isEmpty())
         return "";
      StringBuilder xml = new StringBuilder();
      for (int i = 0; i < links.size(); i++)
      {
         Link link = links.get(i);
         for (String other : link.excludeBodies)
            xml.append("    <exclude body1=\"").append(link.bodyName).append("\" body2=\"").append(other).append("\"/>\n");
         for (int j = i + 1; j < links.size(); j++)
         {
            Link other = links.get(j);
            if (!sharesBody(link.excludeBodies, other.excludeBodies))
               continue;
            xml.append("    <exclude body1=\"").append(link.bodyName)
               .append("\" body2=\"").append(other.bodyName).append("\"/>\n");
         }
      }
      return xml.toString();
   }

   public void writePoses(Robot robot, mjModel model, mjData data)
   {
      if (links.isEmpty() || robot == null || model == null || model.isNull())
         return;
      try
      {
         writePosesUnsafe(robot, model, data);
      }
      catch (RuntimeException exception)
      {
         if (!poseErrorLogged)
         {
            poseErrorLogged = true;
            LogTools.error("Ignored-joint collisions were not posed: {}", exception.getMessage());
         }
      }
   }

   private void writePosesUnsafe(Robot robot, mjModel model, mjData data)
   {
      // Update only the gripper and its finger children. Walking the whole robot from the root
      // follows kinematic loops and overflows, which aborts the physics step and drops every contact.
      for (String anchorLinkName : anchorLinkNames)
      {
         SimRigidBodyBasics anchor = robot.getRigidBody(anchorLinkName);
         if (anchor != null && anchor.getParentJoint() != null)
            anchor.getParentJoint().updateFramesRecursively();
      }
      for (Link link : links)
      {
         int bodyId = Mujoco.mj_name2id(model, Mujoco.mjOBJ_BODY, link.bodyName);
         if (bodyId < 0)
            continue;
         int mocapId = model.body_mocapid().get(bodyId);
         if (mocapId < 0)
            continue;
         SimRigidBodyBasics body = robot.getRigidBody(link.linkName);
         if (body == null || body.getParentJoint() == null)
            continue;
         linkPose.setFromReferenceFrame(body.getParentJoint().getFrameAfterJoint());
         linkPose.changeFrame(ReferenceFrame.getWorldFrame());
         quaternion.set(linkPose.getOrientation());
         data.mocap_pos().put(mocapId * 3L, linkPose.getPosition().getX());
         data.mocap_pos().put(mocapId * 3L + 1, linkPose.getPosition().getY());
         data.mocap_pos().put(mocapId * 3L + 2, linkPose.getPosition().getZ());
         data.mocap_quat().put(mocapId * 4L, quaternion.getS());
         data.mocap_quat().put(mocapId * 4L + 1, quaternion.getX());
         data.mocap_quat().put(mocapId * 4L + 2, quaternion.getY());
         data.mocap_quat().put(mocapId * 4L + 3, quaternion.getZ());
      }
   }

   private String geomXml(String geomName, CollisionShapeDefinition shape)
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
         size = "type=\"mesh\" mesh=\"" + geomName + "_mesh\"";
      }
      else
      {
         return null;
      }

      String pose = "";
      origin.set(shape.getOriginPose());
      if (origin.hasTranslation() || origin.hasRotation())
      {
         quaternion.set(origin.getRotation());
         pose = " pos=\"" + fmt(origin.getTranslation().getX()) + " "
                + fmt(origin.getTranslation().getY()) + " "
                + fmt(origin.getTranslation().getZ()) + "\" quat=\""
                + fmt(quaternion.getS()) + " " + fmt(quaternion.getX()) + " "
                + fmt(quaternion.getY()) + " " + fmt(quaternion.getZ()) + "\"";
      }
      return "      <geom class=\"robot\" name=\"" + geomName + "\"" + pose + " " + size + "/>\n";
   }

   private static boolean sharesBody(Set<String> a, Set<String> b)
   {
      for (String name : a)
      {
         if (b.contains(name))
            return true;
      }
      return false;
   }

   private static String fmt(double value)
   {
      return String.format(Locale.US, "%.8g", value);
   }

   private static final class Link
   {
      private final String linkName;
      private final String bodyName;
      private final Set<String> excludeBodies;

      private Link(String linkName, String bodyName, Set<String> excludeBodies)
      {
         this.linkName = linkName;
         this.bodyName = bodyName;
         this.excludeBodies = excludeBodies;
      }
   }
}
