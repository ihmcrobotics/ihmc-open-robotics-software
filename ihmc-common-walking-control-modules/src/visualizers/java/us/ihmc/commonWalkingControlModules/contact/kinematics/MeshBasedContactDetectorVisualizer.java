package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

public class MeshBasedContactDetectorVisualizer
{
   private static final double contactThreshold = 0.04;

   public static void main(String[] args)
   {
      visualizeSphereContact();
//      visualizeCylinderContact();
//      visualizeCapsuleContact();
//      visualizeBoxContact();
   }

   private static void visualizeSphereContact()
   {
      double radius = 0.2;
      AppearanceDefinition appearance = YoAppearance.DarkGreen();
      appearance.setTransparency(0.3);
      RobotDescription sphereRobot = ContactDetectorTestTools.newSphereRobot("sphere", radius, 1.0, radius, appearance);
      String linkName = "sphereLink";
      RobotCollisionModel collisionModel = RobotCollisionModel.singleBodyCollisionModel(linkName, body -> new Collidable(body, -1, -1, new FrameSphere3D(body.getBodyFixedFrame(), radius)));
      RigidBodyBasics idRobot = ContactDetectorTestTools.toInverseDynamicsRobot(sphereRobot);
      SixDoFJoint floatingJoint = (SixDoFJoint) idRobot.getChildrenJoints().get(0);

      YoRegistry registry = new YoRegistry("visualizationRegistry");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      MeshBasedContactDetector contactDetector = new MeshBasedContactDetector(collisionModel.getRobotCollidables(idRobot), graphicsListRegistry, registry);
      RobotFromDescription robot = setupRobot(sphereRobot, idRobot, floatingJoint, contactDetector);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.addYoRegistry(registry);

      setContactGraphics(contactDetector, scs);

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private static void visualizeCylinderContact()
   {
      double radius = 0.1;
      double height = 0.4;
      AppearanceDefinition appearance = YoAppearance.DarkGreen();
      appearance.setTransparency(0.3);
      RobotDescription cylinderRobot = ContactDetectorTestTools.newCylinderRobot("cylinder", radius, height, 1.0, 0.1, appearance);
      String linkName = "cylinderLink";
      RobotCollisionModel collisionModel = RobotCollisionModel.singleBodyCollisionModel(linkName, body -> new Collidable(body, -1, -1, new FrameCylinder3D(body.getBodyFixedFrame(), height, radius)));
      RigidBodyBasics idRobot = ContactDetectorTestTools.toInverseDynamicsRobot(cylinderRobot);
      SixDoFJoint floatingJoint = (SixDoFJoint) idRobot.getChildrenJoints().get(0);

      YoRegistry registry = new YoRegistry("visualizationRegistry");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      MeshBasedContactDetector contactDetector = new MeshBasedContactDetector(collisionModel.getRobotCollidables(idRobot), graphicsListRegistry, registry);

      RobotFromDescription robot = setupRobot(cylinderRobot, idRobot, floatingJoint, contactDetector);

      FloatingJoint scsFloatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
      scsFloatingJoint.getOrientation().set(new YawPitchRoll(0.4, 1.2, -0.3));

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.addYoRegistry(registry);

      setContactGraphics(contactDetector, scs);
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private static void visualizeCapsuleContact()
   {
      double radius = 0.1;
      double height = 0.4;
      AppearanceDefinition appearance = YoAppearance.DarkGreen();
      appearance.setTransparency(0.3);
      RobotDescription capsuleRobot = ContactDetectorTestTools.newCapsuleRobot("capsule", radius, height, 1.0, 0.1, appearance);
      String linkName = "capsuleLink";
      RobotCollisionModel collisionModel = RobotCollisionModel.singleBodyCollisionModel(linkName, body -> new Collidable(body, -1, -1, new FrameCapsule3D(body.getBodyFixedFrame(), height, radius)));
      RigidBodyBasics idRobot = ContactDetectorTestTools.toInverseDynamicsRobot(capsuleRobot);
      SixDoFJoint floatingJoint = (SixDoFJoint) idRobot.getChildrenJoints().get(0);

      YoRegistry registry = new YoRegistry("visualizationRegistry");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      MeshBasedContactDetector contactDetector = new MeshBasedContactDetector(collisionModel.getRobotCollidables(idRobot), graphicsListRegistry, registry);

      RobotFromDescription robot = setupRobot(capsuleRobot, idRobot, floatingJoint, contactDetector);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.addYoRegistry(registry);

      setContactGraphics(contactDetector, scs);
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private static void visualizeBoxContact()
   {
      double sizeX = 0.24;
      double sizeY = 0.3;
      double sizeZ = 0.1;
      AppearanceDefinition appearance = YoAppearance.DarkGreen();
      appearance.setTransparency(0.3);
      RobotDescription boxRobot = ContactDetectorTestTools.newBoxRobot("box", sizeX, sizeY, sizeZ, 1.00, 0.3, appearance);
      String linkName = "boxLink";
      RobotCollisionModel collisionModel = RobotCollisionModel.singleBodyCollisionModel(linkName, body -> new Collidable(body, -1, -1, new FrameBox3D(body.getBodyFixedFrame(), sizeX, sizeY, sizeZ)));
      RigidBodyBasics idRobot = ContactDetectorTestTools.toInverseDynamicsRobot(boxRobot);
      SixDoFJoint floatingJoint = (SixDoFJoint) idRobot.getChildrenJoints().get(0);

      YoRegistry registry = new YoRegistry("visualizationRegistry");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      MeshBasedContactDetector contactDetector = new MeshBasedContactDetector(collisionModel.getRobotCollidables(idRobot), graphicsListRegistry, registry);
      RobotFromDescription robot = setupRobot(boxRobot, idRobot, floatingJoint, contactDetector);

      FloatingJoint scsFloatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
      scsFloatingJoint.getOrientation().set(new YawPitchRoll(0.4, 1.2, -0.3));

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.addYoRegistry(registry);

      setContactGraphics(contactDetector, scs);
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private static void setContactGraphics(MeshBasedContactDetector contactDetector, SimulationConstructionSet scs)
   {
      scs.setGroundVisible(false);
      scs.setSimulateNoFasterThanRealTime(true);

      Graphics3DObject contactPlaneGraphics = new Graphics3DObject();
      contactPlaneGraphics.translate(0.0, 0.0, contactThreshold);
      ConvexPolygon2D contactPlanePolygon = new ConvexPolygon2D();
      contactPlanePolygon.addVertex(-3.0, -3.0);
      contactPlanePolygon.addVertex(-3.0, 3.0);
      contactPlanePolygon.addVertex(3.0, -3.0);
      contactPlanePolygon.addVertex(3.0, 3.0);
      contactPlanePolygon.update();
      AppearanceDefinition gray = YoAppearance.Gray();
      gray.setTransparency(0.2);
      contactPlaneGraphics.addExtrudedPolygon(contactPlanePolygon, 0.002, gray);
      scs.addStaticLinkGraphics(contactPlaneGraphics);

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

   static RobotFromDescription setupRobot(RobotDescription robotDescription,
                                           RigidBodyBasics idRobot,
                                           SixDoFJoint floatingJoint,
                                           MeshBasedContactDetector contactDetector)
   {
      RobotFromDescription robot = new RobotFromDescription(robotDescription);
      FloatingJoint scsFloatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
      robot.setDynamic(false);
      scsFloatingJoint.getPosition().setZ(0.2);

      YoRegistry registry = new YoRegistry("readerReg");
      Random random = new Random(390);

      YoBoolean setToRandomOrientation = new YoBoolean("setToRandomOrientation", registry);
      RobotController reader = new RobotController()
      {
         @Override
         public void doControl()
         {
            if (setToRandomOrientation.getValue())
            {
               setToRandomOrientation.set(false);
               scsFloatingJoint.getOrientation().set(EuclidCoreRandomTools.nextOrientation3D(random));
            }

            floatingJoint.getJointPose().set(scsFloatingJoint.getPosition(), scsFloatingJoint.getOrientation());
            idRobot.updateFramesRecursively();
            contactDetector.update();
         }

         @Override
         public void initialize()
         {

         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return registry;
         }
      };
      robot.setController(reader);
      return robot;
   }
}