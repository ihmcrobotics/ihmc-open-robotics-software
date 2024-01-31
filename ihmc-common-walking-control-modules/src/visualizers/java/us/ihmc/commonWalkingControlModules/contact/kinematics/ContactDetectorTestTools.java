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
import us.ihmc.scs2.definition.geometry.*;
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
   static RobotDefinition newSphereRobot(String name, double radius, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition("floatingJoint");
      RigidBodyDefinition sphere = newSphereLink(name + "Link", radius, mass, radiusOfGyrationPercent, color);

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

   static RobotDefinition newCylinderRobot(String name, double radius, double length, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition("floatingJoint");
      RigidBodyDefinition cylinder = newCylinderLink(name + "Link", radius, length, mass, radiusOfGyrationPercent, color);

      RobotDefinition robotDescription = new RobotDefinition(name);
      robotDescription.setRootBodyDefinition(elevator);
      elevator.addChildJoint(floatingJoint);
      floatingJoint.setSuccessor(cylinder);

      return robotDescription;
   }

   public static RigidBodyDefinition newCylinderLink(String name, double radius, double length, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition cylinder = new RigidBodyDefinition(name);

      cylinder.setMass(mass);
      double mR = radiusOfGyrationPercent * radius;
      double mL = radiusOfGyrationPercent * length;
      cylinder.getMomentOfInertia().setToDiagonal(mR, mR, mL);

      GeometryDefinition geometryDefinition = new Cylinder3DDefinition(length, radius);
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      cylinder.addVisualDefinition(new VisualDefinition(geometryDefinition, materialDefinition));
      cylinder.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryDefinition));

      return cylinder;
   }

   static RobotDefinition newCapsuleRobot(String name, double radius, double length, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition("floatingJoint");
      RigidBodyDefinition capsule = newCapsuleLink(name + "Link", radius, length, mass, radiusOfGyrationPercent, color);

      RobotDefinition robotDescription = new RobotDefinition(name);
      robotDescription.setRootBodyDefinition(elevator);
      elevator.addChildJoint(floatingJoint);
      floatingJoint.setSuccessor(capsule);

      return robotDescription;
   }

   static RigidBodyDefinition newCapsuleLink(String name, double radius, double length, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition capsule = new RigidBodyDefinition(name);

      capsule.setMass(mass);
      double mR = radiusOfGyrationPercent * radius;
      double mL = radiusOfGyrationPercent * length;
      capsule.getMomentOfInertia().setToDiagonal(mR, mR, mL);

      GeometryDefinition geometryDefinition = new Capsule3DDefinition(length, radius);
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      capsule.addVisualDefinition(new VisualDefinition(geometryDefinition, materialDefinition));
      capsule.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryDefinition));

      return capsule;
   }

   static RobotDefinition newBoxRobot(String name, Tuple3DReadOnly size, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      return newBoxRobot(name, size.getX(), size.getY(), size.getZ(), mass, radiusOfGyrationPercent, color);
   }

   static RobotDefinition newBoxRobot(String name, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition("floatingJoint");
      RigidBodyDefinition box = newBoxLink(name + "Link", sizeX, sizeY, sizeZ, mass, radiusOfGyrationPercent, color);

      RobotDefinition robotDescription = new RobotDefinition(name);
      robotDescription.setRootBodyDefinition(elevator);
      elevator.addChildJoint(floatingJoint);
      floatingJoint.setSuccessor(box);

      return robotDescription;
   }

   static RigidBodyDefinition newBoxLink(String name, double sizeX, double sizeY, double sizeZ, double mass, double radiusOfGyrationPercent, ColorDefinition color)
   {
      RigidBodyDefinition box = new RigidBodyDefinition(name);

      box.setMass(mass);
      box.getMomentOfInertia().setToDiagonal(radiusOfGyrationPercent * sizeX, radiusOfGyrationPercent * sizeY, radiusOfGyrationPercent * sizeZ);

      GeometryDefinition geometryDefinition = new Box3DDefinition(sizeX, sizeY, sizeZ);
      MaterialDefinition materialDefinition = new MaterialDefinition(color);
      box.addVisualDefinition(new VisualDefinition(geometryDefinition, materialDefinition));
      box.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryDefinition));

      return box;
   }
}
