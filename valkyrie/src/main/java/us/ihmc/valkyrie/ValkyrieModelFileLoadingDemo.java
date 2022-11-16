package us.ihmc.valkyrie;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationToolkit.RobotDefinitionTools;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieModelFileLoadingDemo
{
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = true;
   private static final boolean SHOW_KINEMATICS_COLLISIONS = false;
   private static final boolean SHOW_SIM_COLLISIONS = false;

   private SimulationConstructionSet2 scs;

   public ValkyrieModelFileLoadingDemo()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.DEFAULT);
      RobotDefinition robotDefinition = robotModel.getRobotDefinition();

      if (SHOW_KINEMATICS_COLLISIONS)
      {
         RobotCollisionModel collisionModel = robotModel.getHumanoidRobotKinematicsCollisionModel();
         if (collisionModel != null)
            RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.createFullRobotModel().getElevator()),
                                                                robotDefinition);
      }

      if (SHOW_SIM_COLLISIONS)
      {
         RobotCollisionModel collisionModel = robotModel.getSimulationRobotCollisionModel(new CollidableHelper(), "robot", "ground");
         if (collisionModel != null)
            RobotDefinitionTools.addCollisionsToRobotDefinition(collisionModel.getRobotCollidables(robotModel.createFullRobotModel().getElevator()),
                                                                robotDefinition);
      }

      Robot robot = new Robot(robotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);

      YoGraphicGroupDefinition extraViz = new YoGraphicGroupDefinition("ExtraVisualization", new ArrayList<>());

      if (SHOW_COORDINATES_AT_JOINT_ORIGIN)
         extraViz.addChild(createJointFrameVisualization(robot));

      scs = new SimulationConstructionSet2(PhysicsEngineFactory.newDoNothingPhysicsEngineFactory());
      scs.addRobot(robot);
      scs.addYoGraphic(extraViz);
      scs.start(true, false, false);
   }

   public static YoGraphicDefinition createJointFrameVisualization(Robot robot)
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition("Joint Frames");
      for (JointBasics joint : robot.getAllJoints())
      {
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(joint.getName() + " - FrameAfterJoint",
                                                                                  new FramePose3D(joint.getFrameAfterJoint()),
                                                                                  0.5,
                                                                                  ColorDefinitions.Brown()));
      }
      return group;
   }

   // This is for external use
   public static void addKinematicsCollisionGraphics(FullHumanoidRobotModel fullRobotModel, us.ihmc.simulationconstructionset.Robot robot, RobotCollisionModel collisionModel)
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
      new ValkyrieModelFileLoadingDemo();
   }

}
