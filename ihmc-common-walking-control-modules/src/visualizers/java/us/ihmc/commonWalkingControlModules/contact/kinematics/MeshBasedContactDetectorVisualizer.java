package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.function.Function;
import java.util.stream.Collectors;

public class MeshBasedContactDetectorVisualizer
{
   private static final double contactThreshold = 0.04;
   private static final ColorDefinition color = ColorDefinitions.DarkGreen();

   static
   {
      color.setAlpha(0.3);
   }

   public static void main(String[] args)
   {
//      visualizeSphereContact();
//      visualizeCylinderContact();
      visualizeCapsuleContact();
//      visualizeBoxContact();
   }

   private static void visualizeSphereContact()
   {
      double radius = 0.2;
      RobotDefinition sphereRobot = ContactDetectorTestTools.newSphereRobot("sphere", radius, 1.0, radius, color);
      Function<RigidBodyBasics, Collidable> collidableBuilder = body -> new Collidable(body, -1, -1, new FrameSphere3D(body.getBodyFixedFrame(), radius));

      runVisualizer(sphereRobot, collidableBuilder);
   }


   private static void visualizeCylinderContact()
   {
      double radius = 0.15;
      double length = 0.3;
      RobotDefinition cylinderRobot = ContactDetectorTestTools.newCylinderRobot("cylinder", radius, length, 1.0, radius, color);
      Function<RigidBodyBasics, Collidable> collidableBuilder = body -> new Collidable(body, -1, -1, new FrameCylinder3D(body.getBodyFixedFrame(), length, radius));

      runVisualizer(cylinderRobot, collidableBuilder);
   }

   private static void visualizeCapsuleContact()
   {
      double radius = 0.1;
      double length = 0.4;
      RobotDefinition capsuleRobot = ContactDetectorTestTools.newCapsuleRobot("capsule", radius, length, 1.0, 0.1, color);
      Function<RigidBodyBasics, Collidable> collidableBuilder = body -> new Collidable(body, -1, -1, new FrameCapsule3D(body.getBodyFixedFrame(), length, radius));

      runVisualizer(capsuleRobot, collidableBuilder);
   }

   private static void visualizeBoxContact()
   {
      double sizeX = 0.1;
      double sizeY = 0.2;
      double sizeZ = 0.25;
      RobotDefinition boxRobot = ContactDetectorTestTools.newBoxRobot("box", sizeX, sizeY, sizeZ, 1.0, 0.1, color);
      Function<RigidBodyBasics, Collidable> collidableBuilder = body -> new Collidable(body, -1, -1, new FrameBox3D(body.getBodyFixedFrame(), sizeX, sizeY, sizeZ));

      runVisualizer(boxRobot, collidableBuilder);
   }

   private static void runVisualizer(RobotDefinition robotDefinition, Function<RigidBodyBasics, Collidable> collidableBuilder)
   {
      String linkName = robotDefinition.getName() + "Link";
      RobotCollisionModel collisionModel = RobotCollisionModel.singleBodyCollisionModel(linkName, collidableBuilder);

      robotDefinition.ignoreAllJoints();
      RigidBodyBasics rootBody = robotDefinition.newInstance(ReferenceFrame.getWorldFrame());
      MeshBasedContactDetector contactDetector = new MeshBasedContactDetector(collisionModel.getRobotCollidables(rootBody));

      SimulationSession simulationSession = new SimulationSession();
      Robot robot = simulationSession.getPhysicsEngine().addRobot(robotDefinition);
      robot.addController(setupContactController(robot, rootBody, contactDetector, simulationSession.getRootRegistry()));
      setContactGraphics(contactDetector, simulationSession);
      simulationSession.getRootRegistry().addChild(contactDetector.getRegistry());
      simulationSession.addYoGraphicDefinition(contactDetector.getSCS2YoGraphics());

      SessionVisualizer.startSessionVisualizer(simulationSession);
   }

//
//   private static void visualizeBoxContact()
//   {
//      double sizeX = 0.24;
//      double sizeY = 0.3;
//      double sizeZ = 0.1;
//      AppearanceDefinition appearance = YoAppearance.DarkGreen();
//      appearance.setTransparency(0.3);
//      RobotDescription boxRobot = ContactDetectorTestTools.newBoxRobot("box", sizeX, sizeY, sizeZ, 1.00, 0.3, appearance);
//      String linkName = "boxLink";
//      RobotCollisionModel collisionModel = RobotCollisionModel.singleBodyCollisionModel(linkName, body -> new Collidable(body, -1, -1, new FrameBox3D(body.getBodyFixedFrame(), sizeX, sizeY, sizeZ)));
//      RigidBodyBasics idRobot = ContactDetectorTestTools.toInverseDynamicsRobot(boxRobot);
//      SixDoFJoint floatingJoint = (SixDoFJoint) idRobot.getChildrenJoints().get(0);
//
//      YoRegistry registry = new YoRegistry("visualizationRegistry");
//      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
//      MeshBasedContactDetector contactDetector = new MeshBasedContactDetector(collisionModel.getRobotCollidables(idRobot), graphicsListRegistry, registry);
//      RobotFromDescription robot = setupRobot(boxRobot, idRobot, floatingJoint, contactDetector);
//
//      FloatingJoint scsFloatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
//      scsFloatingJoint.getOrientation().set(new YawPitchRoll(0.4, 1.2, -0.3));
//
//      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
//      scs.addYoGraphicsListRegistry(graphicsListRegistry);
//      scs.addYoRegistry(registry);
//
//      setContactGraphics(contactDetector, scs);
//      scs.startOnAThread();
//      ThreadTools.sleepForever();
//   }

   private static void setContactGraphics(MeshBasedContactDetector contactDetector, SimulationSession simulation)
   {
      simulation.getSimulationSessionControls().setRealTimeRateSimulation(true);

      Box3DDefinition contactPlane = new Box3DDefinition(3.0, 3.0, 0.002);
      ColorDefinition gray = ColorDefinitions.Gray();
      gray.setAlpha(0.8);
      VisualDefinition environmentVisual = new VisualDefinition(contactPlane, gray);
      CollisionShapeDefinition boxCollisionShape = new CollisionShapeDefinition(contactPlane);
      boxCollisionShape.getOriginPose().getTranslation().setZ(contactThreshold);

      simulation.addTerrainObject(new TerrainObjectDefinition(environmentVisual, boxCollisionShape));

      List<FrameShape3DBasics> environmentShapes = List.of(new FrameBox3D(ReferenceFrame.getWorldFrame(), new Point3D(0.0, 0.0, -0.5), new Quaternion(), 10.0, 10.0, 1.0));
      contactDetector.setEnvironmentShapes(environmentShapes);
      contactDetector.setContactThreshold(contactThreshold);
   }

   private static MeshDataHolder newConvexPolytope3DMesh(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      MeshDataBuilder meshBuilder = new MeshDataBuilder();

      for (Face3DReadOnly face : convexPolytope3D.getFaces())
      {
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         meshBuilder.addMesh(MeshDataGenerator.Polygon(ccwFaceVertices));
      }

      return meshBuilder.generateMeshDataHolder();
   }

   static Controller setupContactController(Robot robot, RigidBodyBasics rootBody, MeshBasedContactDetector contactDetector, YoRegistry registry)
   {
      SixDoFJoint rootJoint = (SixDoFJoint) rootBody.getChildrenJoints().get(0);
      SimFloatingJointBasics floatingJoint = robot.getFloatingRootJoint();
      floatingJoint.getJointPose().getPosition().setZ(0.2);

      Random random = new Random(390);

      YoBoolean setToRandomOrientation = new YoBoolean("setToRandomOrientation", registry);
      return () ->
      {
         if (setToRandomOrientation.getValue())
         {
            setToRandomOrientation.set(false);
            floatingJoint.getJointPose().getOrientation().set(EuclidCoreRandomTools.nextOrientation3D(random));
         }

         rootJoint.getJointPose().set(floatingJoint.getJointPose().getPosition(), floatingJoint.getJointPose().getOrientation());
         rootBody.updateFramesRecursively();
         contactDetector.update();
      };
   }
}