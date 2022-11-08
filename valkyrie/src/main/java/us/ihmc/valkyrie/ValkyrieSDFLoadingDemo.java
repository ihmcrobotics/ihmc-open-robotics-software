package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

/**
 * Created by dstephen on 2/7/14.
 */
public class ValkyrieSDFLoadingDemo
{
   private static final boolean SHOW_ELLIPSOIDS = false;
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = false;
   private static final boolean SHOW_INERTIA_ELLIPSOIDS = false;
   private static final boolean SHOW_KINEMATICS_COLLISIONS = false;
   private static final boolean SHOW_SIM_COLLISIONS = true;

   private SimulationConstructionSet scs;

   public ValkyrieSDFLoadingDemo()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.ARM_MASS_SIM);

      FloatingRootJointRobot valkyrieRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      valkyrieRobot.setPositionInWorld(new Vector3D());

      if (SHOW_ELLIPSOIDS)
      {
         addIntertialEllipsoidsToVisualizer(valkyrieRobot);
      }

      if (SHOW_COORDINATES_AT_JOINT_ORIGIN)
         addJointAxis(valkyrieRobot);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      if (SHOW_INERTIA_ELLIPSOIDS)
      {
         CommonInertiaEllipsoidsVisualizer inertiaVis = new CommonInertiaEllipsoidsVisualizer(fullRobotModel.getElevator(), yoGraphicsListRegistry);
         inertiaVis.update();
      }

      if (SHOW_KINEMATICS_COLLISIONS)
         addKinematicsCollisionGraphics(fullRobotModel, valkyrieRobot, robotModel.getHumanoidRobotKinematicsCollisionModel());

      if (SHOW_SIM_COLLISIONS)
      {
         RobotCollisionModel collisionModel = robotModel.getSimulationRobotCollisionModel(new CollidableHelper(), "robot", "ground");
         addKinematicsCollisionGraphics(fullRobotModel, valkyrieRobot, collisionModel);
      }

      scs = new SimulationConstructionSet(valkyrieRobot);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   private void addIntertialEllipsoidsToVisualizer(FloatingRootJointRobot valkyrieRobot)
   {
      ArrayList<Joint> joints = new ArrayList<>();
      joints.add(valkyrieRobot.getRootJoint());

      HashSet<Link> links = getAllLinks(joints, new HashSet<Link>());

      for (Link l : links)
      {
         AppearanceDefinition appearance = YoAppearance.Green();
         appearance.setTransparency(0.6);
         l.addEllipsoidFromMassProperties(appearance);
         l.addCoordinateSystemToCOM(0.5);
         //         l.addBoxFromMassProperties(appearance);
      }
   }

   private HashSet<Link> getAllLinks(List<Joint> joints, HashSet<Link> links)
   {
      for (Joint j : joints)
      {
         links.add(j.getLink());

         if (!j.getChildrenJoints().isEmpty())
         {
            links.addAll(getAllLinks(j.getChildrenJoints(), links));
         }
      }

      return links;
   }

   public static void addJointAxis(FloatingRootJointRobot valkyrieRobot)
   {

      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>(Arrays.asList(valkyrieRobot.getOneDegreeOfFreedomJoints()));

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.5);
         linkGraphics.combine(joint.getLink().getLinkGraphics());
         joint.getLink().setLinkGraphics(linkGraphics);
      }
   }

   public static void addKinematicsCollisionGraphics(FullHumanoidRobotModel fullRobotModel, Robot robot, RobotCollisionModel collisionModel)
   {
      List<Collidable> robotCollidables = collisionModel.getRobotCollidables(fullRobotModel.getElevator());

      for (Collidable collidable : robotCollidables)
      {
         Link link = robot.getLink(collidable.getRigidBody().getName());
         link.getLinkGraphics().combine(getGraphics(collidable));
      }
   }

   private static Graphics3DObject getGraphics(Collidable collidable)
   {
      Shape3DReadOnly shape = collidable.getShape();
      RigidBodyTransform transformToParentJoint = collidable.getShape().getReferenceFrame()
                                                            .getTransformToDesiredFrame(collidable.getRigidBody().getParentJoint().getFrameAfterJoint());
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.transform(transformToParentJoint);
      AppearanceDefinition appearance = YoAppearance.DarkGreen();
      appearance.setTransparency(0.5);

      if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         graphics.translate(sphere.getPosition());
         graphics.addSphere(sphere.getRadius(), appearance);
      }
      else if (shape instanceof Capsule3DReadOnly)
      {
         Capsule3DReadOnly capsule = (Capsule3DReadOnly) shape;
         RigidBodyTransform transform = new RigidBodyTransform();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule.getAxis(), transform.getRotation());
         transform.getTranslation().set(capsule.getPosition());
         graphics.transform(transform);
         graphics.addCapsule(capsule.getRadius(),
                             capsule.getLength() + 2.0 * capsule.getRadius(), // the 2nd term is removed internally.
                             appearance);
      }
      else if (shape instanceof Box3DReadOnly)
      {
         Box3DReadOnly box = (Box3DReadOnly) shape;
         graphics.translate(box.getPosition());
         graphics.rotate(new RotationMatrix(box.getOrientation()));
         graphics.addCube(box.getSizeX(), box.getSizeY(), box.getSizeZ(), true, appearance);
      }
      else if (shape instanceof PointShape3DReadOnly)
      {
         PointShape3DReadOnly pointShape = (PointShape3DReadOnly) shape;
         graphics.translate(pointShape);
         graphics.addSphere(0.01, appearance);
      }
      else if (shape instanceof FrameEllipsoid3DReadOnly)
      {
         FrameEllipsoid3DReadOnly ellipsoidShape = (FrameEllipsoid3DReadOnly) shape;
         graphics.transform(new RigidBodyTransform(ellipsoidShape.getPose()));
         graphics.addEllipsoid(ellipsoidShape.getRadiusX(), ellipsoidShape.getRadiusY(), ellipsoidShape.getRadiusZ(), appearance);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape.getClass().getSimpleName());
      }
      return graphics;
   }

   public static void main(String[] args)
   {
      new ValkyrieSDFLoadingDemo();
   }

}
