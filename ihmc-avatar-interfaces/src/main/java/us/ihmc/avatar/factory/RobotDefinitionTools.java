package us.ihmc.avatar.factory;

import java.io.InputStream;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Predicate;

import javax.xml.bind.JAXBException;

import org.apache.commons.lang3.tuple.ImmutablePair;

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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPBox3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCapsule3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPConvexPolytope3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCylinder3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPRamp3DReadOnly;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Point3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCapsule3DDefinition;
import us.ihmc.scs2.definition.geometry.STPConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.STPRamp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;

public class RobotDefinitionTools
{
   public static RobotDefinition loadSDFModel(InputStream stream,
                                              Collection<String> resourceDirectories,
                                              ClassLoader classLoader,
                                              String modelName,
                                              ContactPointDefinitionHolder contactPointDefinitionHolder,
                                              JointNameMap<?> jointNameMap)
   {
      try
      {
         SDFRoot sdfRoot = SDFTools.loadSDFRoot(stream, resourceDirectories, classLoader);
         RobotDefinition robotDefinition = SDFTools.toFloatingRobotDefinition(sdfRoot, modelName);

         if (contactPointDefinitionHolder != null)
            addGroundContactPoints(robotDefinition, contactPointDefinitionHolder);

         if (jointNameMap != null)
         {
            for (String jointName : jointNameMap.getLastSimulatedJoints())
               robotDefinition.addSubtreeJointsToIgnore(jointName);
            adjustJointLimitStops(robotDefinition, jointNameMap);
         }
         adjustRigidBodyInterias(robotDefinition);

         return robotDefinition;
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void setDefaultMaterial(RobotDefinition robotDefinition)
   {
      setDefaultMaterial(robotDefinition, new MaterialDefinition(ColorDefinitions.Orange().derive(0, 1, 1, 0.4)));
   }

   public static void setDefaultMaterial(RobotDefinition robotDefinition, MaterialDefinition defaultMaterial)
   {
      for (RigidBodyDefinition rigidBodyDefinition : robotDefinition.getAllRigidBodies())
      {
         for (VisualDefinition visualDefinition : rigidBodyDefinition.getVisualDefinitions())
         {
            if (visualDefinition.getMaterialDefinition() != null)
               continue;
            GeometryDefinition geometryDefinition = visualDefinition.getGeometryDefinition();
            if (geometryDefinition instanceof ModelFileGeometryDefinition
                  && !((ModelFileGeometryDefinition) geometryDefinition).getFileName().toLowerCase().endsWith(".stl"))
               continue;
            visualDefinition.setMaterialDefinition(defaultMaterial);
         }
      }
   }

   public static void removeCollisionShapeDefinitions(RobotDefinition robotDefinition)
   {
      for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
         body.getCollisionShapeDefinitions().clear();
   }

   public static void setRobotDefinitionMaterial(RobotDefinition robotDefinition, MaterialDefinition materialDefinition)
   {
      for (RigidBodyDefinition body : robotDefinition.getAllRigidBodies())
      {
         body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(materialDefinition));
      }
   }

   public static void setRobotDefinitionTransparency(RobotDefinition robotDefinition, double transparency)
   {
      setRobotDefinitionMaterial(robotDefinition, new MaterialDefinition(ColorDefinitions.Orange().derive(0, 1, 1, 1.0 - transparency)));
   }

   public static void adjustJointLimitStops(RobotDefinition robotDefinition, JointNameMap<?> jointNameMap)
   {
      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         if (joint instanceof RevoluteJointDefinition)
         {
            RevoluteJointDefinition revoluteJoint = (RevoluteJointDefinition) joint;

            if (isJointInNeedOfReducedGains(joint.getName()))
            {
               revoluteJoint.setKpSoftLimitStop(10.0);
               revoluteJoint.setKdSoftLimitStop(2.5);
            }
            else if (revoluteJoint.getKpSoftLimitStop() <= 0.0 && revoluteJoint.getKdSoftLimitStop() <= 0.0)
            {
               revoluteJoint.setKpSoftLimitStop(jointNameMap.getJointKLimit(joint.getName()));
               revoluteJoint.setKdSoftLimitStop(jointNameMap.getJointBLimit(joint.getName()));
            }
            else
            {
               revoluteJoint.setKpSoftLimitStop(0.0001 * revoluteJoint.getKpSoftLimitStop());
               revoluteJoint.setKdSoftLimitStop(0.1 * revoluteJoint.getKdSoftLimitStop());
            }
         }
         else if (joint instanceof PrismaticJointDefinition)
         {
            PrismaticJointDefinition prismaticJoint = (PrismaticJointDefinition) joint;

            if (isJointInNeedOfReducedGains(joint.getName()))
            {
               prismaticJoint.setKpSoftLimitStop(100.0);
               prismaticJoint.setKdSoftLimitStop(20.0);
            }
            else if (prismaticJoint.getKpSoftLimitStop() <= 0.0 && prismaticJoint.getKdSoftLimitStop() <= 0.0)
            {
               prismaticJoint.setKpSoftLimitStop(jointNameMap.getJointKLimit(joint.getName()));
               prismaticJoint.setKdSoftLimitStop(jointNameMap.getJointBLimit(joint.getName()));
            }
            else
            {
               prismaticJoint.setKpSoftLimitStop(0.0001 * prismaticJoint.getKpSoftLimitStop());
               prismaticJoint.setKdSoftLimitStop(prismaticJoint.getKdSoftLimitStop());
            }
         }
      }
   }

   public static void adjustRigidBodyInterias(RobotDefinition robotDefinition)
   {
      for (JointDefinition joint : robotDefinition.getAllJoints())
      {
         if (isJointInNeedOfReducedGains(joint.getName()))
         {
            RigidBodyDefinition successor = joint.getSuccessor();
            successor.getMomentOfInertia().scale(100.0);
         }
      }
   }

   private static boolean isJointInNeedOfReducedGains(String jointName)
   {
      return jointName.contains("f0") || jointName.contains("f1") || jointName.contains("f2") || jointName.contains("f3") || jointName.contains("palm")
            || jointName.contains("finger");
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder)
   {
      addGroundContactPoints(robotDefinition, contactPointHolder, true);
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder, boolean addVisualization)
   {
      if (contactPointHolder == null)
         return;

      LinkedHashMap<String, Integer> counters = new LinkedHashMap<String, Integer>();
      for (ImmutablePair<String, Vector3D> jointContactPoint : contactPointHolder.getJointNameGroundContactPointMap())
      {
         String jointName = jointContactPoint.getLeft();

         int count;
         if (counters.get(jointName) == null)
            count = 0;
         else
            count = counters.get(jointName);

         Vector3D gcOffset = jointContactPoint.getRight();

         GroundContactPointDefinition groundContactPoint = new GroundContactPointDefinition();
         groundContactPoint.setName("gc_" + ModelFileLoaderConversionsHelper.sanitizeJointName(jointName) + "_" + count++);
         groundContactPoint.getTransformToParent().getTranslation().set(gcOffset);
         groundContactPoint.setGroupIdentifier(contactPointHolder.getGroupIdentifier(jointContactPoint));

         JointDefinition jointDefinition = robotDefinition.getJointDefinition(jointName);

         jointDefinition.addGroundContactPointDefinition(groundContactPoint);

         counters.put(jointName, count);

         if (addVisualization)
         {
            VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
            visualDefinitionFactory.appendTranslation(jointContactPoint.getRight());
            visualDefinitionFactory.addSphere(0.01, new MaterialDefinition(ColorDefinitions.Orange()));
            jointDefinition.getSuccessor().getVisualDefinitions().addAll(visualDefinitionFactory.getVisualDefinitions());
         }
      }
   }

   public static void scaleRobotDefinition(RobotDefinition definition, double modelScale, double massScalePower, Predicate<JointDefinition> jointFilter)
   {
      scaleRigidBodyDefinitionRecursive(definition.getRootBodyDefinition(), modelScale, massScalePower, jointFilter, true);
   }

   private static void scaleRigidBodyDefinitionRecursive(RigidBodyDefinition definition,
                                                         double modelScale,
                                                         double massScalePower,
                                                         Predicate<JointDefinition> jointFilter,
                                                         boolean scaleInertia)
   {
      if (scaleInertia && definition.getParentJoint() != null)
         scaleInertia &= jointFilter.test(definition.getParentJoint());

      scaleRigidBodyDefinition(definition, modelScale, massScalePower, scaleInertia);

      for (JointDefinition joint : definition.getChildrenJoints())
      {
         scaleJointDefinition(joint, modelScale);
         scaleRigidBodyDefinitionRecursive(joint.getSuccessor(), modelScale, massScalePower, jointFilter, scaleInertia);
      }
   }

   private static void scaleJointDefinition(JointDefinition definition, double modelScale)
   {
      definition.getTransformToParent().getTranslation().scale(modelScale);

      for (SensorDefinition sensor : definition.getSensorDefinitions())
         sensor.getTransformToJoint().getTranslation().scale(modelScale);
      for (KinematicPointDefinition kp : definition.getKinematicPointDefinitions())
         kp.getTransformToParent().getTranslation().scale(modelScale);
      for (ExternalWrenchPointDefinition efp : definition.getExternalWrenchPointDefinitions())
         efp.getTransformToParent().getTranslation().scale(modelScale);
      for (GroundContactPointDefinition gcp : definition.getGroundContactPointDefinitions())
         gcp.getTransformToParent().getTranslation().scale(modelScale);
   }

   private static void scaleRigidBodyDefinition(RigidBodyDefinition definition, double modelScale, double massScalePower, boolean scaleInertia)
   {
      // Center of mass offset scales with the scaling factor
      definition.getCenterOfMassOffset().scale(modelScale);

      // Mass scales with factor^massScalePower. massScalePower is 3 when considering constant density

      if (scaleInertia)
      {
         double massScale = Math.pow(modelScale, massScalePower);
         definition.setMass(massScale * definition.getMass());

         if (definition.getMomentOfInertia() != null)
         {
            // The components of the inertia matrix are defined with int(r^2 dm). So they scale factor ^ (2 + massScalePower)
            double inertiaScale = Math.pow(modelScale, massScalePower + 2);
            definition.getMomentOfInertia().scale(inertiaScale);
         }
      }

      for (VisualDefinition visual : definition.getVisualDefinitions())
         visual.getOriginPose().appendScale(modelScale);
      for (CollisionShapeDefinition collision : definition.getCollisionShapeDefinitions())
         scaleCollisionShapeDefinition(collision, modelScale);
   }

   private static void scaleCollisionShapeDefinition(CollisionShapeDefinition definition, double scale)
   {
      throw new RuntimeException("TODO: Implement me");
   }

   public static Consumer<RobotDefinition> jointLimitRemover()
   {
      return jointLimitRemover(null);
   }

   public static Consumer<RobotDefinition> jointLimitRemover(String nameFilter)
   {
      return jointLimitMutator(nameFilter, -100.0, 100.0);
   }

   public static Consumer<RobotDefinition> jointLimitMutator(String nameFilter, double lowerLimit, double upperLimit)
   {
      return robotDefinition ->
      {
         for (JointDefinition joint : robotDefinition.getAllJoints())
         {
            String jointName = joint.getName();
            if (!(joint instanceof OneDoFJointDefinition))
               continue;
            if (nameFilter != null && !nameFilter.isEmpty() && !jointName.contains(nameFilter))
               continue;

            ((OneDoFJointDefinition) joint).setPositionLimits(lowerLimit, upperLimit);
         }
      };
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

   public static CollisionShapeDefinition toCollisionShapeDefinition(Collidable source)
   {
      RigidBodyTransform pose = new RigidBodyTransform();
      GeometryDefinition geometry = null;
      CollisionShapeDefinition output = new CollisionShapeDefinition();
      output.setCollisionMask(source.getCollisionMask());
      output.setCollisionGroup(source.getCollisionGroup());

      FrameShape3DReadOnly shape = source.getShape();

      if (shape instanceof FrameSTPBox3DReadOnly)
      {
         FrameSTPBox3DReadOnly stpBox3D = (FrameSTPBox3DReadOnly) shape;
         STPBox3DDefinition stpGeometry = new STPBox3DDefinition(stpBox3D.getSize());
         stpGeometry.setMargins(stpBox3D.getMinimumMargin(), stpBox3D.getMaximumMargin());
         pose.set(shape.getPose());
         geometry = stpGeometry;
      }
      else if (shape instanceof FrameBox3DReadOnly)
      {
         FrameBox3DReadOnly box3D = (FrameBox3DReadOnly) shape;
         geometry = new Box3DDefinition(box3D.getSize());
         pose.set(shape.getPose());
      }
      else if (shape instanceof FrameSTPCapsule3DReadOnly)
      {
         FrameSTPCapsule3DReadOnly stpCapsule3D = (FrameSTPCapsule3DReadOnly) shape;
         STPCapsule3DDefinition stpGeometry = new STPCapsule3DDefinition(stpCapsule3D.getLength(), stpCapsule3D.getRadius());
         stpGeometry.setMargins(stpCapsule3D.getMinimumMargin(), stpCapsule3D.getMaximumMargin());
         geometry = stpGeometry;
      }
      else if (shape instanceof FrameCapsule3DReadOnly)
      {
         FrameCapsule3DReadOnly capsule3D = (FrameCapsule3DReadOnly) shape;
         geometry = new Capsule3DDefinition(capsule3D.getLength(), capsule3D.getRadius());
         pose.getTranslation().set(capsule3D.getPosition());
         EuclidGeometryTools.orientation3DFromZUpToVector3D(capsule3D.getAxis(), pose.getRotation());
      }
      else if (shape instanceof FrameSTPConvexPolytope3DReadOnly)
      {
         FrameSTPConvexPolytope3DReadOnly stpConvexPolytope3D = (FrameSTPConvexPolytope3DReadOnly) shape;
         STPConvexPolytope3DDefinition stpGeometry = new STPConvexPolytope3DDefinition(stpConvexPolytope3D);
         stpGeometry.setMargins(stpConvexPolytope3D.getMinimumMargin(), stpConvexPolytope3D.getMaximumMargin());
         geometry = stpGeometry;
      }
      else if (shape instanceof FrameConvexPolytope3DReadOnly)
      {
         FrameConvexPolytope3DReadOnly convexPolytope3D = (FrameConvexPolytope3DReadOnly) shape;
         geometry = new ConvexPolytope3DDefinition(convexPolytope3D);
      }
      else if (shape instanceof FrameSTPCylinder3DReadOnly)
      {
         FrameSTPCylinder3DReadOnly stpCylinder3D = (FrameSTPCylinder3DReadOnly) shape;
         STPCylinder3DDefinition stpGeometry = new STPCylinder3DDefinition(stpCylinder3D.getLength(), stpCylinder3D.getRadius());
         stpGeometry.setMargins(stpCylinder3D.getMinimumMargin(), stpCylinder3D.getMaximumMargin());
         geometry = stpGeometry;
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
      else if (shape instanceof FrameSTPRamp3DReadOnly)
      {
         FrameSTPRamp3DReadOnly stpRamp3D = (FrameSTPRamp3DReadOnly) shape;
         STPRamp3DDefinition stpGeometry = new STPRamp3DDefinition(stpRamp3D.getSize());
         stpGeometry.setMargins(stpRamp3D.getMinimumMargin(), stpRamp3D.getMaximumMargin());
         geometry = stpGeometry;
      }
      else if (shape instanceof FrameRamp3DReadOnly)
      {
         FrameRamp3DReadOnly ramp3D = (FrameRamp3DReadOnly) shape;
         geometry = new Ramp3DDefinition(ramp3D.getSize());
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

      output.setOriginPose(pose);
      output.setGeometryDefinition(geometry);

      return output;
   }
}
