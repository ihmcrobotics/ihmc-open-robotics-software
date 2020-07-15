package us.ihmc.atlas;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import us.ihmc.atlas.parameters.AtlasSimulationCollisionModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.CollidableVisualizer;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.tools.inputDevices.keyboard.ModifierKeyInterface;

public class AtlasSDFViewer
{
   private static final boolean SHOW_ELLIPSOIDS = false;
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = false;
   private static final boolean SHOW_IMU_FRAMES = false;
   private static final boolean SHOW_KINEMATICS_COLLISIONS = false;
   private static final boolean SHOW_SIM_COLLISIONS = true;

   public static void main(String[] args)
   {
      AtlasRobotVersion atlasRobotVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      DRCRobotModel robotModel = new AtlasRobotModel(atlasRobotVersion, RobotTarget.SCS, false);
      HumanoidFloatingRootJointRobot sdfRobot = robotModel.createHumanoidFloatingRootJointRobot(false);

      if (SHOW_ELLIPSOIDS)
      {
         addIntertialEllipsoidsToVisualizer(sdfRobot);
      }

      if (SHOW_COORDINATES_AT_JOINT_ORIGIN)
      {
         addJointAxis(sdfRobot);
      }

      if (SHOW_IMU_FRAMES)
      {
         showIMUFrames(sdfRobot);
      }

      if (SHOW_KINEMATICS_COLLISIONS)
      {
         FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
         fullRobotModel.updateFrames();
         addKinematicsCollisionGraphics(fullRobotModel, sdfRobot, robotModel.getHumanoidRobotKinematicsCollisionModel());
      }

      if (SHOW_SIM_COLLISIONS)
      {
         FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
         fullRobotModel.updateFrames();
         FramePoint3D chestCoM = new FramePoint3D(fullRobotModel.getChest().getBodyFixedFrame());
         chestCoM.changeFrame(ReferenceFrame.getWorldFrame());
         System.out.println(chestCoM);
         AtlasSimulationCollisionModel collisionModel = new AtlasSimulationCollisionModel(robotModel.getJointMap(), atlasRobotVersion);
         collisionModel.setCollidableHelper(new CollidableHelper(), "robot", "ground");
         addKinematicsCollisionGraphics(fullRobotModel, sdfRobot, collisionModel);
      }

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);

      SelectedListener selectedListener = new SelectedListener()
      {
         @Override
         public void selected(Graphics3DNode graphics3dNode, ModifierKeyInterface modifierKeyInterface, Point3DReadOnly location,
                              Point3DReadOnly cameraLocation, QuaternionReadOnly cameraRotation)
         {
            System.out.println("Clicked location " + location);
         }
      };

      scs.attachSelectedListener(selectedListener);

      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   private static void showIMUFrames(HumanoidFloatingRootJointRobot sdfRobot)
   {
      ArrayList<IMUMount> imuMounts = new ArrayList<>();
      sdfRobot.getIMUMounts(imuMounts);

      for (IMUMount imuMount : imuMounts)
      {
         Link imuLink = imuMount.getParentJoint().getLink();
         if (imuLink.getLinkGraphics() == null)
            imuLink.setLinkGraphics(new Graphics3DObject());

         Graphics3DObject linkGraphics = imuLink.getLinkGraphics();
         linkGraphics.identity();
         RigidBodyTransform mountToJoint = new RigidBodyTransform();
         imuMount.getTransformFromMountToJoint(mountToJoint);
         linkGraphics.transform(mountToJoint);
         linkGraphics.addCoordinateSystem(0.3);
         linkGraphics.identity();
      }
   }

   private static void addIntertialEllipsoidsToVisualizer(FloatingRootJointRobot sdfRobot)
   {
      ArrayList<Joint> joints = new ArrayList<>();
      joints.add(sdfRobot.getRootJoint());

      HashSet<Link> links = getAllLinks(joints, new HashSet<Link>());

      for (Link link : links)
      {
         if (link.getLinkGraphics() == null)
            link.setLinkGraphics(new Graphics3DObject());

         AppearanceDefinition appearance = YoAppearance.Green();
         appearance.setTransparency(0.6);
         link.addEllipsoidFromMassProperties(appearance);
         link.addCoordinateSystemToCOM(0.1);
      }
   }

   private static HashSet<Link> getAllLinks(List<Joint> joints, HashSet<Link> links)
   {
      for (Joint joint : joints)
      {
         links.add(joint.getLink());

         if (!joint.getChildrenJoints().isEmpty())
         {
            links.addAll(getAllLinks(joint.getChildrenJoints(), links));
         }
      }

      return links;
   }

   public static void addJointAxis(FloatingRootJointRobot sdfRobot)
   {
      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>(Arrays.asList(sdfRobot.getOneDegreeOfFreedomJoints()));

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.3);
         if (joint.getLink().getLinkGraphics() != null)
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
         Graphics3DObject linkGraphics = link.getLinkGraphics();
         if (linkGraphics == null)
         {
            linkGraphics = new Graphics3DObject();
            link.setLinkGraphics(linkGraphics);
         }
         linkGraphics.combine(getGraphics(collidable));
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
      else if (shape instanceof ConvexPolytope3DReadOnly)
      {
         ConvexPolytope3DReadOnly convexPolytope = (ConvexPolytope3DReadOnly) shape;
         graphics.addMeshData(CollidableVisualizer.newConvexPolytope3DMesh(convexPolytope), appearance);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape.getClass().getSimpleName());
      }
      return graphics;
   }
}
