package us.ihmc.commonWalkingControlModules.contact.kinematics;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.mecano.multiBodySystem.*;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * From ExampleExperimentalSimulationTools
 */

public class ContactDetectorTestTools
{
   static RigidBodyBasics toInverseDynamicsRobot(RobotDescription description)
   {
      RigidBody rootBody = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      for (JointDescription rootJoint : description.getRootJoints())
         addJointRecursive(rootJoint, rootBody);
      return rootBody;
   }

   static void addJointRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      JointBasics joint;
      String name = jointDescription.getName();
      Vector3D jointOffset = new Vector3D();
      jointDescription.getOffsetFromParentJoint(jointOffset);

      if (jointDescription instanceof PinJointDescription)
      {
         Vector3D jointAxis = new Vector3D();
         ((PinJointDescription) jointDescription).getJointAxis(jointAxis);
         joint = new RevoluteJoint(name, parentBody, jointOffset, jointAxis);
      }
      else if (jointDescription instanceof SliderJointDescription)
      {
         Vector3D jointAxis = new Vector3D();
         ((SliderJointDescription) jointDescription).getJointAxis(jointAxis);
         joint = new PrismaticJoint(name, parentBody, jointOffset, jointAxis);
      }
      else if (jointDescription instanceof FloatingJointDescription)
      {
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.getTranslation().set(jointOffset);
         joint = new SixDoFJoint(name, parentBody, transformToParent);
      }
      else if (jointDescription instanceof BallAndSocketJointDescription)
      {
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.getTranslation().set(jointOffset);
         joint = new SphericalJoint(name, parentBody, transformToParent);
      }
      else
      {
         throw new IllegalStateException("Joint type not handled.");
      }

      LinkDescription linkDescription = jointDescription.getLink();

      String bodyName = linkDescription.getName();
      Matrix3DReadOnly momentOfInertia = linkDescription.getMomentOfInertiaCopy();
      double mass = linkDescription.getMass();
      Tuple3DReadOnly centerOfMassOffset = linkDescription.getCenterOfMassOffset();
      RigidBody successor = new RigidBody(bodyName, joint, momentOfInertia, mass, centerOfMassOffset);
      joint.setSuccessor(successor);

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addJointRecursive(childJoint, successor);
   }

   static RobotDefinition newSphereRobot(String name, double radius, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition("floatingJoint");
      RigidBodyDefinition sphere = newSphereLink("sphereLink", radius, mass, radiusOfGyrationPercent, color);

      RobotDefinition robotDescription = new RobotDefinition(name);
      robotDescription.setRootBodyDefinition(elevator);
      elevator.addChildJoint(floatingJoint);
      floatingJoint.setSuccessor(sphere);

      return robotDescription;
   }

   public static RigidBodyDefinition newSphereLink(String name, double radius, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition sphere = new RigidBodyDefinition(name);

      sphere.setMass(mass);
      double radiusOfGyration = radiusOfGyrationPercent * radius;
      sphere.getMomentOfInertia().setToDiagonal(radiusOfGyration, radiusOfGyration, radiusOfGyration);

      GeometryDefinition geometryDefinition = new Sphere3DDefinition(radius);
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      sphere.addVisualDefinition(new VisualDefinition(geometryDefinition, materialDefinition));
      sphere.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryDefinition));

      return sphere;
   }

   static RobotDescription newCylinderRobot(String name, double radius, double height, double mass, double radiusOfGyrationPercent, AppearanceDefinition appearance)
   {
      RobotDescription robotDescription = new RobotDescription(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription(name, name);
      LinkDescription link = new LinkDescription(name + "Link");
      link.setMassAndRadiiOfGyration(mass, radiusOfGyrationPercent * radius, radiusOfGyrationPercent * radius, radiusOfGyrationPercent * height);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.translate(0.0, 0.0, -height / 2.0);
      linkGraphics.addCylinder(height, radius, appearance);

      link.setLinkGraphics(linkGraphics);
      rootJoint.setLink(link);
      robotDescription.addRootJoint(rootJoint);

      return robotDescription;
   }

   static RobotDescription newCapsuleRobot(String name, double radius, double height, double mass, double radiusOfGyrationPercent, AppearanceDefinition appearance)
   {
      RobotDescription robotDescription = new RobotDescription(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription(name, name);
      LinkDescription link = new LinkDescription(name + "Link");
      link.setMassAndRadiiOfGyration(mass, radiusOfGyrationPercent * radius, radiusOfGyrationPercent * radius, radiusOfGyrationPercent * height);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      linkGraphics.addCapsule(radius, height + 2.0 * radius, appearance);

      link.setLinkGraphics(linkGraphics);
      rootJoint.setLink(link);
      robotDescription.addRootJoint(rootJoint);

      return robotDescription;
   }

   static RobotDescription newBoxRobot(String name, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent, AppearanceDefinition appearance)
   {
      return newBoxRobot(name, size.getX(), size.getY(), size.getZ(), mass, radiusOfGyrationPercent, appearance);
   }

   static RobotDescription newBoxRobot(String name, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent,
                                       AppearanceDefinition appearance)
   {
      RobotDescription robotDescription = new RobotDescription(name);

      FloatingJointDescription rootJoint = new FloatingJointDescription(name, name);
      rootJoint.setLink(newBoxLink(name + "Link", sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, appearance));
      robotDescription.addRootJoint(rootJoint);

      return robotDescription;
   }

   public static LinkDescription newBoxLink(String linkName, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent, AppearanceDefinition appearance)
   {
      return newBoxLink(linkName, size, mass, radiusOfGyrationPercent, null, appearance);
   }

   public static LinkDescription newBoxLink(String linkName, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent,
                                            Vector3DReadOnly offsetFromParent, AppearanceDefinition appearance)
   {
      return newBoxLink(linkName, size.getX(), size.getY(), size.getZ(), mass, radiusOfGyrationPercent, offsetFromParent, appearance);
   }

   public static LinkDescription newBoxLink(String linkName, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent,
                                            AppearanceDefinition appearance)
   {
      return newBoxLink(linkName, sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, null, appearance);
   }

   public static LinkDescription newBoxLink(String linkName, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent,
                                            Vector3DReadOnly offsetFromParentJoint, AppearanceDefinition appearance)
   {
      LinkDescription link = new LinkDescription(linkName);
      link.setMassAndRadiiOfGyration(mass, radiusOfGyrationPercent * sizeX, radiusOfGyrationPercent * sizeY, radiusOfGyrationPercent * sizeZ);
      if (offsetFromParentJoint != null)
         link.setCenterOfMassOffset(offsetFromParentJoint);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
      if (offsetFromParentJoint != null)
         linkGraphics.translate(offsetFromParentJoint);
      linkGraphics.translate(0.0, 0.0, -sizeZ / 2.0);
      linkGraphics.addCube(sizeX, sizeY, sizeZ, appearance);
      link.setLinkGraphics(linkGraphics);
      return link;
   }

   public static LinkGraphicsDescription translateLinkGraphicsDescription(LinkGraphicsDescription input, Vector3DReadOnly translation)
   {
      LinkGraphicsDescription output = new LinkGraphicsDescription();
      output.combine(input, translation);
      return output;
   }

   public static Graphics3DObject translateGraphics3DObject(Graphics3DObject input, Vector3DReadOnly translation)
   {
      Graphics3DObject output = new Graphics3DObject();
      output.combine(input, translation);
      return output;
   }

   static void configureSCSToTrackRobotRootJoint(SimulationConstructionSet scs, RobotDescription robotToTrack)
   {
      JointDescription rootJointToTrack = robotToTrack.getRootJoints().get(0);
      if (rootJointToTrack instanceof FloatingJointDescription)
         configureSCSToTrackFloatingJoint(scs, (FloatingJointDescription) rootJointToTrack);
      else
         return;
   }

   static void configureSCSToTrackFloatingJoint(SimulationConstructionSet scs, FloatingJointDescription floatingJointDescription)
   {
      String varName = floatingJointDescription.getJointVariableName();
      if (varName == null)
         varName = "";
      else if (!varName.isEmpty())
         varName += "_";

      String xVarName = "q_" + varName + "x";
      String yVarName = "q_" + varName + "y";
      String zVarName = "q_" + varName + "z";
      scs.setCameraTrackingVars(null, xVarName, yVarName, zVarName);
   }

   static Graphics3DObject toGraphics3DObject(FrameShape3DReadOnly shape, ReferenceFrame graphicsFrame, AppearanceDefinition appearance)
   {
      return toGraphics3DObject(shape, shape.getReferenceFrame(), graphicsFrame, appearance);
   }

   static Graphics3DObject toGraphics3DObject(Shape3DReadOnly shape, ReferenceFrame shapeFrame, ReferenceFrame graphicsFrame, AppearanceDefinition appearance)
   {
      Graphics3DObject graphics = new Graphics3DObject();
      graphics.transform(shapeFrame.getTransformToDesiredFrame(graphicsFrame));

      if (shape instanceof Sphere3DReadOnly)
      {
         Sphere3DReadOnly sphere = (Sphere3DReadOnly) shape;
         graphics.translate(sphere.getPosition());
         graphics.addSphere(sphere.getRadius(), appearance);
      }
      else if (shape instanceof Cylinder3DReadOnly)
      {
         Cylinder3DReadOnly cylinder = (Cylinder3DReadOnly) shape;
         RigidBodyTransform transform = new RigidBodyTransform();
         EuclidGeometryTools.orientation3DFromZUpToVector3D(cylinder.getAxis(), transform.getRotation());
         transform.getTranslation().set(cylinder.getPosition());
         graphics.transform(transform);
         graphics.translate(0.0, 0.0, -0.5 * cylinder.getLength());
         graphics.addCylinder(cylinder.getLength(), cylinder.getRadius(), appearance);
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
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape.getClass().getSimpleName());
      }
      return graphics;
   }
}
