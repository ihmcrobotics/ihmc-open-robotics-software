package us.ihmc.avatar.factory;

import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.robotDescription.ExternalForcePointDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.KinematicPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SensorDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Point3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.Wedge3DDefinition;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;

public class RobotDefinitionTools
{
   public static RobotDefinition toRobotDefinition(RobotDescription robotDescription)
   {
      RigidBodyDefinition rootBody = new RigidBodyDefinition(robotDescription.getName() + "RootBody");

      for (JointDescription rootJointToCopy : robotDescription.getRootJoints())
         createAndJointsRecursive(rootBody, rootJointToCopy);

      RobotDefinition robotDefinition = new RobotDefinition(robotDescription.getName());
      robotDefinition.setRootBodyDefinition(rootBody);
      return robotDefinition;
   }

   public static void addCollisionsToRobotDefinition(List<Collidable> collidables, RobotDefinition robotDefinition)
   {
      for (Collidable collidable : collidables)
      {
         if (collidable.getRigidBody() == null)
            continue;

         RigidBodyDefinition rigidBodyDefinition = robotDefinition.getRigidBodyDefinition(collidable.getRigidBody().getName());
         rigidBodyDefinition.addCollisionShapeDefinition(toCollisionShapeDefinition(collidable));
      }
   }

   public static void addInitialStateToRobotDefinition(Robot robotAtInitialState, RobotDefinition robotDefinition)
   {
      for (Joint rootJoint : robotAtInitialState.getRootJoints())
      {
         JointDefinition rootJointDefinition = robotDefinition.getRootJointDefinitions().stream().filter(j -> j.getName().equals(rootJoint.getName()))
                                                              .findFirst().get();
         addInitialStateRecursive(rootJoint, rootJointDefinition);
      }
   }

   private static void addInitialStateRecursive(Joint jointAtInitialState, JointDefinition jointDefinition)
   {
      if (jointAtInitialState instanceof FloatingJoint)
      {
         SixDoFJointState initialJointState = new SixDoFJointState();
         initialJointState.setConfiguration(((FloatingJoint) jointAtInitialState).getOrientation(), ((FloatingJoint) jointAtInitialState).getPosition());
         jointDefinition.setInitialJointState(initialJointState);
      }
      else if (jointAtInitialState instanceof OneDegreeOfFreedomJoint)
      {
         OneDoFJointState initialJointState = new OneDoFJointState();
         initialJointState.setConfiguration(((OneDegreeOfFreedomJoint) jointAtInitialState).getQ());
         initialJointState.setVelocity(((OneDegreeOfFreedomJoint) jointAtInitialState).getQD());
         jointDefinition.setInitialJointState(initialJointState);
      }

      for (Joint childJoint : jointAtInitialState.getChildrenJoints())
      {
         JointDefinition childJointDefinition = jointDefinition.getSuccessor().getChildrenJoints().stream()
                                                               .filter(j -> j.getName().equals(childJoint.getName())).findFirst().get();
         addInitialStateRecursive(childJoint, childJointDefinition);
      }
   }

   public static void createAndJointsRecursive(RigidBodyDefinition parentBody, JointDescription jointToCopy)
   {
      JointDefinition jointDefinition = toJointDefinition(jointToCopy);
      parentBody.addChildJoint(jointDefinition);

      RigidBodyDefinition rigidBodyDefinition = toRigidBodyDefinition(jointToCopy.getLink());
      jointDefinition.setSuccessor(rigidBodyDefinition);

      for (JointDescription childJointToCopy : jointToCopy.getChildrenJoints())
         createAndJointsRecursive(rigidBodyDefinition, childJointToCopy);
   }

   public static RigidBodyDefinition toRigidBodyDefinition(LinkDescription source)
   {
      RigidBodyDefinition output = new RigidBodyDefinition(source.getName());
      output.setMass(source.getMass());
      output.setMomentOfInertia(source.getMomentOfInertiaCopy());
      output.setCenterOfMassOffset(source.getCenterOfMassOffset());
      return output;
   }

   public static JointDefinition toJointDefinition(JointDescription source)
   {
      if (source instanceof FloatingJointDescription)
         return toSixDoFJointDefinition((FloatingJointDescription) source);
      if (source instanceof PinJointDescription)
         return toRevoluteJointDefinition((PinJointDescription) source);
      if (source instanceof SliderJointDescription)
         return toPrismaticJointDefinition((SliderJointDescription) source);
      return null;
   }

   public static SixDoFJointDefinition toSixDoFJointDefinition(FloatingJointDescription source)
   {
      SixDoFJointDefinition output = new SixDoFJointDefinition();
      copyJointProperties(source, output);
      return output;
   }

   public static RevoluteJointDefinition toRevoluteJointDefinition(PinJointDescription source)
   {
      RevoluteJointDefinition output = new RevoluteJointDefinition();
      copyOneDofJointProperties(source, output);
      return output;
   }

   public static PrismaticJointDefinition toPrismaticJointDefinition(SliderJointDescription source)
   {
      PrismaticJointDefinition output = new PrismaticJointDefinition();
      copyOneDofJointProperties(source, output);
      return output;
   }

   private static void copyJointProperties(JointDescription source, JointDefinition destination)
   {
      destination.setName(source.getName());
      destination.getTransformToParent().getTranslation().set(source.getOffsetFromParentJoint());
      source.getIMUSensors().forEach(imu -> destination.addSensorDefinition(toIMUSensorDefinition(imu)));
      source.getForceSensors().forEach(forceSensor -> destination.addSensorDefinition(toWrenchSensorDefinition(forceSensor)));

      source.getKinematicPoints().forEach(kp -> destination.addKinematicPointDefinition(toKinematicPointDefinition(kp)));
      source.getExternalForcePoints().forEach(efp -> destination.addExternalWrenchPointDefinition(toExternalWrenchPointDefinition(efp)));
      source.getGroundContactPoints().forEach(gcp -> destination.addGroundContactPointDefinition(toGroundContactPointDefinition(gcp)));
   }

   private static void copyOneDofJointProperties(OneDoFJointDescription source, OneDoFJointDefinition destination)
   {
      copyJointProperties(source, destination);
      destination.setAxis(source.getJointAxis());
      destination.setPositionLimits(source.getLowerLimit(), source.getUpperLimit());
      destination.setVelocityLimits(source.getVelocityLimit());
      destination.setEffortLimits(source.getEffortLimit());
      destination.setStiction(source.getStiction());
      destination.setDamping(source.getDamping());
   }

   public static SensorDefinition toSensorDefinition(SensorDescription source)
   {
      if (source instanceof IMUSensorDescription)
         return toIMUSensorDefinition((IMUSensorDescription) source);
      if (source instanceof ForceSensorDescription)
         return toWrenchSensorDefinition((ForceSensorDescription) source);
      // TODO Copy other sensors
      return null;
   }

   private static IMUSensorDefinition toIMUSensorDefinition(IMUSensorDescription source)
   {
      IMUSensorDefinition output = new IMUSensorDefinition();
      copySensorProperties(source, output);
      //TODO Copy noise parameters
      return output;
   }

   private static WrenchSensorDefinition toWrenchSensorDefinition(ForceSensorDescription source)
   {
      WrenchSensorDefinition output = new WrenchSensorDefinition();
      copySensorProperties(source, output);
      return output;
   }

   private static void copySensorProperties(SensorDescription source, SensorDefinition destination)
   {
      destination.setName(source.getName());
      destination.getTransformToJoint().set(source.getTransformToJoint());
   }

   private static KinematicPointDefinition toKinematicPointDefinition(KinematicPointDescription source)
   {
      return new KinematicPointDefinition(source.getName(), source.getOffsetFromJoint());
   }

   private static ExternalWrenchPointDefinition toExternalWrenchPointDefinition(ExternalForcePointDescription source)
   {
      return new ExternalWrenchPointDefinition(source.getName(), source.getOffsetFromJoint());
   }

   private static GroundContactPointDefinition toGroundContactPointDefinition(GroundContactPointDescription source)
   {
      return new GroundContactPointDefinition(source.getName(), source.getOffsetFromJoint(), source.getGroupIdentifier());
   }

   public static CollisionShapeDefinition toCollisionShapeDefinition(Collidable source)
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      GeometryDefinition geometry = null;
      CollisionShapeDefinition output = new CollisionShapeDefinition();
      output.setOriginPose(pose);

      FrameShape3DReadOnly shape = source.getShape();

      if (shape instanceof FrameBox3DReadOnly)
      {
         FrameBox3DReadOnly box3D = (FrameBox3DReadOnly) shape;
         geometry = new Box3DDefinition(box3D.getSize());
         pose.set(shape.getPose());
      }
      else if (shape instanceof FrameCapsule3DReadOnly)
      {
         FrameCapsule3DReadOnly capsule3D = (FrameCapsule3DReadOnly) shape;
         geometry = new Capsule3DDefinition(capsule3D.getLength(), capsule3D.getRadius());
         pose.getTranslation().set(capsule3D.getPosition());
         EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule3D.getAxis(), pose.getRotation());
      }
      else if (shape instanceof FrameConvexPolytope3DReadOnly)
      {
         FrameConvexPolytope3DReadOnly convexPolytope3D = (FrameConvexPolytope3DReadOnly) shape;
         geometry = new ConvexPolytope3DDefinition(convexPolytope3D);
      }
      else if (shape instanceof FrameCylinder3DReadOnly)
      {
         FrameCylinder3DReadOnly cylinder3D = (FrameCylinder3DReadOnly) shape;
         geometry = new Capsule3DDefinition(cylinder3D.getLength(), cylinder3D.getRadius());
         pose.getTranslation().set(cylinder3D.getPosition());
         EuclidGeometryTools.orientation3DFromZUpToVector3D(cylinder3D.getAxis(), pose.getRotation());
      }
      else if (shape instanceof FrameEllipsoid3DReadOnly)
      {
         FrameEllipsoid3DReadOnly ellipsoid3D = (FrameEllipsoid3DReadOnly) shape;
         geometry = new Ellipsoid3DDefinition(ellipsoid3D.getRadii());
         pose.set(ellipsoid3D.getPose());
      }
      else if (shape instanceof FramePointShape3DReadOnly)
      {
         FramePointShape3DReadOnly pointShape3D = (FramePointShape3DReadOnly) shape;
         geometry = new Point3DDefinition(pointShape3D);
      }
      else if (shape instanceof FrameRamp3DReadOnly)
      {
         FrameRamp3DReadOnly ramp3D = (FrameRamp3DReadOnly) shape;
         geometry = new Wedge3DDefinition(ramp3D.getSize());
         pose.set(ramp3D.getPose());
      }
      else if (shape instanceof FrameSphere3DReadOnly)
      {
         FrameSphere3DReadOnly sphere3D = (FrameSphere3DReadOnly) shape;
         geometry = new Sphere3DDefinition(sphere3D.getRadius());
         pose.getTranslation().set(sphere3D.getPosition());
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported shape: " + shape);
      }

      if (source.getRigidBody() != null)
      {
         RigidBodyTransform additionalTransform = shape.getReferenceFrame()
                                                       .getTransformToDesiredFrame(source.getRigidBody().getParentJoint().getFrameAfterJoint());
         pose.preMultiply(additionalTransform);
      }

      output.setGeometryDefinition(geometry);

      return output;
   }
}
