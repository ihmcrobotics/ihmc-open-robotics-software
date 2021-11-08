package us.ihmc.avatar.factory;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.xml.bind.JAXBException;

import org.apache.commons.lang3.mutable.MutableObject;
import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.SDFAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.graphicsDescription.color.MutableColor;
import us.ihmc.graphicsDescription.instructions.ArcTorusGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.CapsuleGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.ConeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.CubeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.CylinderGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.EllipsoidGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.ExtrudedPolygonGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddExtrusionInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddHeightMapInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddModelFileInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.instructions.HemiEllipsoidGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.PolygonGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.PrimitiveGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.PyramidCubeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.SphereGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.TruncatedConeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.WedgeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DIdentityInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DRotateInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DTranslateInstruction;
import us.ihmc.log.LogTools;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.robotModels.description.InvertedFourBarJointDefinition;
import us.ihmc.robotModels.description.InvertedFourBarJointDescription;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPBox3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCapsule3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPConvexPolytope3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCylinder3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPRamp3DReadOnly;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.robotDescription.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.ExternalForcePointDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.KinematicPointDescription;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.LoopClosureConstraintDescription;
import us.ihmc.robotics.robotDescription.LoopClosurePinConstraintDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SensorDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.scs2.definition.AffineTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.controller.interfaces.ControllerDefinition;
import us.ihmc.scs2.definition.geometry.ArcTorus3DDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrudedPolygon2DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrusionDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.HemiEllipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition.SubMeshDefinition;
import us.ihmc.scs2.definition.geometry.Point3DDefinition;
import us.ihmc.scs2.definition.geometry.Polygon3DDefinition;
import us.ihmc.scs2.definition.geometry.PyramidBox3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCapsule3DDefinition;
import us.ihmc.scs2.definition.geometry.STPConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.STPRamp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.Torus3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.geometry.TruncatedCone3DDefinition;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.LidarSensorDefinition;
import us.ihmc.scs2.definition.robot.LoopClosureDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.scs2.definition.robot.sdf.SDFTools;
import us.ihmc.scs2.definition.robot.sdf.items.SDFRoot;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
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

   public static RobotDefinition toRobotDefinition(RobotDescription robotDescription)
   {
      RigidBodyDefinition rootBody = new RigidBodyDefinition(robotDescription.getName() + "RootBody");

      for (JointDescription rootJointToCopy : robotDescription.getRootJoints())
         createAndAddJointsRecursive(rootBody, rootJointToCopy);

      List<ControllerDefinition> constraints = new ArrayList<>();
      for (JointDescription rootJointToCopy : robotDescription.getRootJoints())
         createLoopClosureConstraintRecursive(rootJointToCopy, constraints);

      RobotDefinition robotDefinition = new RobotDefinition(robotDescription.getName());
      robotDefinition.setRootBodyDefinition(rootBody);
      constraints.forEach(robotDefinition::addControllerDefinition);
      return robotDefinition;
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

   public static void createAndAddJointsRecursive(RigidBodyDefinition parentBody, JointDescription jointToCopy)
   {
      JointDefinition jointDefinition = toJointDefinition(jointToCopy);
      parentBody.addChildJoint(jointDefinition);

      RigidBodyDefinition rigidBodyDefinition = toRigidBodyDefinition(jointToCopy.getLink());
      jointDefinition.setSuccessor(rigidBodyDefinition);

      for (JointDescription childJointToCopy : jointToCopy.getChildrenJoints())
         createAndAddJointsRecursive(rigidBodyDefinition, childJointToCopy);
   }

   public static RigidBodyDefinition toRigidBodyDefinition(LinkDescription source)
   {
      RigidBodyDefinition output = new RigidBodyDefinition(source.getName());
      output.setMass(source.getMass());
      output.setMomentOfInertia(source.getMomentOfInertiaCopy());
      output.setCenterOfMassOffset(source.getCenterOfMassOffset());
      output.addVisualDefinitions(toVisualDefinitions(source.getLinkGraphics()));
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
      source.getLidarSensors().forEach(lidar -> destination.addSensorDefinition(toLidarSensorDefinition(lidar)));
      source.getCameraSensors().forEach(camera -> destination.addSensorDefinition(toCameraSensorDefinition(camera)));

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
      double[] limitStopParameters = source.getLimitStopParameters();
      destination.setGainsSoftLimitStop(limitStopParameters[2], limitStopParameters[3]);
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

   private static LidarSensorDefinition toLidarSensorDefinition(LidarSensorDescription source)
   {
      LidarSensorDefinition output = new LidarSensorDefinition();
      copySensorProperties(source, output);
      output.setSweepYawMin(source.getSweepYawMin());
      output.setSweepYawMax(source.getSweepYawMax());
      output.setHeightPitchMin(source.getHeightPitchMin());
      output.setHeightPitchMax(source.getHeightPitchMax());
      output.setMinRange(source.getMinRange());
      output.setMaxRange(source.getMaxRange());
      output.setPointsPerSweep(source.getPointsPerSweep());
      output.setScanHeight(source.getScanHeight());
      return output;
   }

   private static CameraSensorDefinition toCameraSensorDefinition(CameraSensorDescription source)
   {
      CameraSensorDefinition output = new CameraSensorDefinition();
      copySensorProperties(source, output);
      // TODO The transform in the description is x-forward and z-up while the camera should be z-forward and y-down
      output.getTransformToJoint().appendYawRotation(-Math.PI / 2.0);
      output.getTransformToJoint().appendRollRotation(-Math.PI / 2.0);
      output.setFieldOfView(source.getFieldOfView());
      output.setClipNear(source.getClipNear());
      output.setClipFar(100000.0);//source.getClipFar()); // TODO Allows to view the entire scene, not sure if that's what we want
      output.setImageWidth(source.getImageWidth());
      output.setImageHeight(source.getImageHeight());
      output.setUpdatePeriod(1000 / 25); // 25Hz // TODO Weird this is not present in the description. 
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

   public static List<VisualDefinition> toVisualDefinitions(Graphics3DObject graphics3DObject)
   {
      if (graphics3DObject == null)
         return Collections.emptyList();

      List<VisualDefinition> visualDefinitions = new ArrayList<>();

      AffineTransform currentTransform = new AffineTransform();

      for (Graphics3DPrimitiveInstruction instruction : graphics3DObject.getGraphics3DInstructions())
      {
         if (instruction instanceof Graphics3DIdentityInstruction)
         {
            currentTransform.setIdentity();
         }
         else if (instruction instanceof Graphics3DRotateInstruction)
         {
            currentTransform.appendOrientation(((Graphics3DRotateInstruction) instruction).getRotationMatrix());
         }
         else if (instruction instanceof Graphics3DScaleInstruction)
         {
            currentTransform.appendScale(((Graphics3DScaleInstruction) instruction).getScaleFactor());
         }
         else if (instruction instanceof Graphics3DTranslateInstruction)
         {
            currentTransform.appendTranslation(((Graphics3DTranslateInstruction) instruction).getTranslation());
         }
         else if (instruction instanceof Graphics3DInstruction)
         {
            VisualDefinition visualDefinition = new VisualDefinition();
            visualDefinition.setOriginPose(new AffineTransform(currentTransform));
            visualDefinition.setMaterialDefinition(toMaterialDefinition(((Graphics3DInstruction) instruction).getAppearance()));
            visualDefinitions.add(visualDefinition);

            if (instruction instanceof PrimitiveGraphics3DInstruction)
            {
               if (instruction instanceof ArcTorusGraphics3DInstruction)
               {
                  ArcTorusGraphics3DInstruction arcTorus = (ArcTorusGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new ArcTorus3DDefinition(arcTorus.getStartAngle(),
                                                                                  arcTorus.getEndAngle(),
                                                                                  arcTorus.getMajorRadius(),
                                                                                  arcTorus.getMinorRadius(),
                                                                                  arcTorus.getResolution()));
               }
               else if (instruction instanceof CapsuleGraphics3DInstruction)
               {
                  CapsuleGraphics3DInstruction capsule = (CapsuleGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new Capsule3DDefinition(capsule.getHeight(),
                                                                                 capsule.getXRadius(),
                                                                                 capsule.getYRadius(),
                                                                                 capsule.getZRadius(),
                                                                                 capsule.getResolution()));
               }
               else if (instruction instanceof ConeGraphics3DInstruction)
               {
                  ConeGraphics3DInstruction cone = (ConeGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new Cone3DDefinition(cone.getHeight(), cone.getRadius(), cone.getResolution()));
               }
               else if (instruction instanceof CubeGraphics3DInstruction)
               {
                  CubeGraphics3DInstruction cube = (CubeGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new Box3DDefinition(cube.getLength(),
                                                                             cube.getWidth(),
                                                                             cube.getHeight(),
                                                                             cube.getCenteredInTheCenter()));
               }
               else if (instruction instanceof CylinderGraphics3DInstruction)
               {
                  CylinderGraphics3DInstruction cylinder = (CylinderGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new Cylinder3DDefinition(cylinder.getHeight(), cylinder.getRadius(), false));
               }
               else if (instruction instanceof EllipsoidGraphics3DInstruction)
               {
                  EllipsoidGraphics3DInstruction ellipsoid = (EllipsoidGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new Ellipsoid3DDefinition(ellipsoid.getXRadius(),
                                                                                   ellipsoid.getYRadius(),
                                                                                   ellipsoid.getZRadius(),
                                                                                   ellipsoid.getResolution()));
               }
               else if (instruction instanceof ExtrudedPolygonGraphics3DInstruction)
               {
                  ExtrudedPolygonGraphics3DInstruction extrusion = (ExtrudedPolygonGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new ExtrudedPolygon2DDefinition(ExtrudedPolygon2DDefinition.toPoint2DDefinitionList(extrusion.getPolygonPoints()),
                                                                                         true,
                                                                                         extrusion.getExtrusionHeight()));
               }
               else if (instruction instanceof HemiEllipsoidGraphics3DInstruction)
               {
                  HemiEllipsoidGraphics3DInstruction hemiEllipsoid = (HemiEllipsoidGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new HemiEllipsoid3DDefinition(hemiEllipsoid.getXRadius(),
                                                                                       hemiEllipsoid.getYRadius(),
                                                                                       hemiEllipsoid.getZRadius(),
                                                                                       hemiEllipsoid.getResolution()));
               }
               else if (instruction instanceof PolygonGraphics3DInstruction)
               {
                  PolygonGraphics3DInstruction polygon = (PolygonGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new Polygon3DDefinition(Polygon3DDefinition.toPoint3DDefinitionList(polygon.getPolygonPoints()),
                                                                                 true));
               }
               else if (instruction instanceof PyramidCubeGraphics3DInstruction)
               {
                  PyramidCubeGraphics3DInstruction pyramid = (PyramidCubeGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new PyramidBox3DDefinition(pyramid.getLengthX(),
                                                                                    pyramid.getWidthY(),
                                                                                    pyramid.getHeightZ(),
                                                                                    pyramid.getPyramidHeight()));
               }
               else if (instruction instanceof SphereGraphics3DInstruction)
               {
                  SphereGraphics3DInstruction sphere = (SphereGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new Sphere3DDefinition(sphere.getRadius(), sphere.getResolution()));
               }
               else if (instruction instanceof TruncatedConeGraphics3DInstruction)
               {
                  TruncatedConeGraphics3DInstruction cone = (TruncatedConeGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new TruncatedCone3DDefinition(cone.getHeight(),
                                                                                       cone.getXTopRadius(),
                                                                                       cone.getYTopRadius(),
                                                                                       cone.getXBaseRadius(),
                                                                                       cone.getYBaseRadius(),
                                                                                       false,
                                                                                       cone.getResolution()));
               }
               else if (instruction instanceof WedgeGraphics3DInstruction)
               {
                  WedgeGraphics3DInstruction wedge = (WedgeGraphics3DInstruction) instruction;
                  visualDefinition.setGeometryDefinition(new Ramp3DDefinition(wedge.getLengthX(), wedge.getWidthY(), wedge.getHeightZ()));
               }
            }
            else if (instruction instanceof Graphics3DAddExtrusionInstruction)
            {
               Graphics3DAddExtrusionInstruction extrusion = (Graphics3DAddExtrusionInstruction) instruction;
               visualDefinition.setGeometryDefinition(new ExtrusionDefinition(extrusion.getBufferedImage(), extrusion.getHeight()));
            }
            else if (instruction instanceof Graphics3DAddHeightMapInstruction)
            {
               throw new UnsupportedOperationException("Unsupported instruction");
            }
            else if (instruction instanceof Graphics3DAddMeshDataInstruction)
            {
               Graphics3DAddMeshDataInstruction mesh = (Graphics3DAddMeshDataInstruction) instruction;
               visualDefinition.setGeometryDefinition(new TriangleMesh3DDefinition(mesh.getMeshData().getName(),
                                                                                   mesh.getMeshData().getVertices(),
                                                                                   Stream.of(mesh.getMeshData().getTexturePoints()).map(Point2D32::new)
                                                                                         .toArray(Point2D32[]::new),
                                                                                   mesh.getMeshData().getVertexNormals(),
                                                                                   mesh.getMeshData().getTriangleIndices()));
            }
            else if (instruction instanceof Graphics3DAddModelFileInstruction)
            {
               Graphics3DAddModelFileInstruction model = (Graphics3DAddModelFileInstruction) instruction;
               ModelFileGeometryDefinition definition = new ModelFileGeometryDefinition();
               definition.setFileName(model.getFileName());
               definition.setResourceClassLoader(model.getResourceClassLoader());
               definition.setResourceDirectories(model.getResourceDirectories());
               definition.setSubmeshes(Collections.singletonList(new SubMeshDefinition(model.getSubmesh(), model.centerSubmesh())));
               visualDefinition.setGeometryDefinition(definition);
            }
            else
            {
               throw new UnsupportedOperationException("Unsupported instruction type: " + instruction);
            }
         }
         else
         {
            throw new UnsupportedOperationException("Unsupported instruction type: " + instruction);
         }
      }

      return visualDefinitions;
   }

   private static MaterialDefinition toMaterialDefinition(AppearanceDefinition appearanceDefinition)
   {
      if (appearanceDefinition == null)
         return null;

      MaterialDefinition output = new MaterialDefinition();

      if (appearanceDefinition instanceof SDFAppearance)
      {

      }
      else if (appearanceDefinition instanceof YoAppearanceMaterial)
      {
         YoAppearanceMaterial yoAppearanceMaterial = (YoAppearanceMaterial) appearanceDefinition;
         output.setDiffuseColor(toColorDefinition(yoAppearanceMaterial.getDiffuseColor(), yoAppearanceMaterial.getTransparency()));
         output.setSpecularColor(toColorDefinition(yoAppearanceMaterial.getSpecularColor(), yoAppearanceMaterial.getTransparency()));
         output.setShininess(yoAppearanceMaterial.getShininess());
         output.setAmbientColor(toColorDefinition(yoAppearanceMaterial.getAmbientColor(), yoAppearanceMaterial.getTransparency()));
      }
      else if (appearanceDefinition instanceof YoAppearanceRGBColor)
      {
         YoAppearanceRGBColor yoAppearanceRGBColor = (YoAppearanceRGBColor) appearanceDefinition;
         output.setDiffuseColor(toColorDefinition(yoAppearanceRGBColor.getColor(), yoAppearanceRGBColor.getTransparency()));
      }
      else if (appearanceDefinition instanceof YoAppearanceTexture)
      {
         YoAppearanceTexture yoAppearanceTexture = (YoAppearanceTexture) appearanceDefinition;
         TextureDefinition textureDefinition = new TextureDefinition();
         textureDefinition.setFilename(yoAppearanceTexture.getPath());
         textureDefinition.setImage(yoAppearanceTexture.getBufferedImage());
         output.setDiffuseMap(textureDefinition);
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported appearance definition type: " + appearanceDefinition);
      }

      return output;
   }

   private static ColorDefinition toColorDefinition(MutableColor mutableColor, double transparency)
   {
      return new ColorDefinition(mutableColor.getX(), mutableColor.getY(), mutableColor.getZ(), 1.0 - transparency);
   }

   private static void createLoopClosureConstraintRecursive(JointDescription jointDescription, List<ControllerDefinition> controllerDefinitionsToPack)
   {
      List<LoopClosureConstraintDescription> constraintDescriptions = jointDescription.getChildrenConstraintDescriptions();

      for (LoopClosureConstraintDescription constraintDescription : constraintDescriptions)
      {
         controllerDefinitionsToPack.add((controllerInput, controllerOutput) ->
         {
            String name = constraintDescription.getName();
            Tuple3DReadOnly offsetFromParentJoint = constraintDescription.getOffsetFromParentJoint();
            Tuple3DReadOnly offsetFromLinkParentJoint = constraintDescription.getOffsetFromLinkParentJoint();
            Matrix3DReadOnly constraintForceSubSpace = constraintDescription.getConstraintForceSubSpace();
            Matrix3DReadOnly constraintMomentSubSpace = constraintDescription.getConstraintMomentSubSpace();
            LoopClosureSoftConstraintSCS2 constraint = new LoopClosureSoftConstraintSCS2(name,
                                                                                         offsetFromParentJoint,
                                                                                         offsetFromLinkParentJoint,
                                                                                         constraintForceSubSpace,
                                                                                         constraintMomentSubSpace);
            constraint.setParentJoint((SimJointBasics) controllerInput.getInput().findJoint(jointDescription.getName()));
            constraint.setRigidBody((SimRigidBodyBasics) controllerInput.getInput().findRigidBody(constraintDescription.getLink().getName()));
            constraint.setGains(constraintDescription.getProportionalGains(), constraintDescription.getDerivativeGains());
            return constraint;
         });
      }

      for (JointDescription childJointDescription : jointDescription.getChildrenJoints())
      {
         createLoopClosureConstraintRecursive(childJointDescription, controllerDefinitionsToPack);
      }
   }

   public static RobotDescription toRobotDescription(RobotDefinition robotDefinition)
   {
      RobotDescription robotDescription = new RobotDescription(robotDefinition.getName());

      for (JointDefinition rootJointDefinition : robotDefinition.getRootJointDefinitions())
         createAndAddJointsRecursive(robotDescription, rootJointDefinition);

      createAndAddLoopClosureJoints(robotDescription, robotDefinition);

      // Fixed the inner joints of four bar joints: The 2 first inner joints needs to be connected to the four-bar parent joint.
      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         if (!(jointDefinition instanceof InvertedFourBarJointDefinition))
            continue;

         InvertedFourBarJointDefinition fourBarDefinition = (InvertedFourBarJointDefinition) jointDefinition;
         InvertedFourBarJointDescription fourBarDescription = (InvertedFourBarJointDescription) robotDescription.getJointDescription(fourBarDefinition.getName());

         for (JointDefinition loopJointDefinition : fourBarDefinition.getFourBarJoints())
         {
            if (loopJointDefinition.isLoopClosure())
               continue;

            if (loopJointDefinition.getParentJoint() == fourBarDefinition.getParentJoint())
            {
               PinJointDescription loopJointDescription = Arrays.stream(fourBarDescription.getFourBarJoints())
                                                                .filter(j -> j.getName().equals(loopJointDefinition.getName())).findFirst().get();
               loopJointDescription.setParentJoint(fourBarDescription.getParentJoint());
            }
         }
      }

      for (String jointName : robotDefinition.getNameOfJointsToIgnore())
         robotDescription.getJointDescription(jointName).setIsDynamic(false);

      return robotDescription;
   }

   public static void createAndAddJointsRecursive(RobotDescription robotDescription, JointDefinition rootJointToCopy)
   {
      JointDescription rootJointDescription = toJointDescription(rootJointToCopy);
      robotDescription.addRootJoint(rootJointDescription);

      LinkDescription linkDescription = toLinkDescription(rootJointToCopy.getSuccessor());
      rootJointDescription.setLink(linkDescription);

      for (JointDefinition childJointToCopy : rootJointToCopy.getSuccessor().getChildrenJoints())
         createAndAddJointsRecursive(rootJointDescription, childJointToCopy);
   }

   public static void createAndAddJointsRecursive(JointDescription parentJoint, JointDefinition jointToCopy)
   {
      if (jointToCopy.isLoopClosure())
         return; // We address the loop closures in a second pass.

      JointDescription jointDescription = toJointDescription(jointToCopy);
      parentJoint.addJoint(jointDescription);

      LinkDescription linkDescription = toLinkDescription(jointToCopy.getSuccessor());
      jointDescription.setLink(linkDescription);

      for (JointDefinition childJointToCopy : jointToCopy.getSuccessor().getChildrenJoints())
         createAndAddJointsRecursive(jointDescription, childJointToCopy);
   }

   private static void createAndAddLoopClosureJoints(RobotDescription robotDescription, RobotDefinition robotDefinition)
   {
      for (JointDefinition jointDefinition : robotDefinition.getAllJoints())
      {
         if (!jointDefinition.isLoopClosure())
            continue;

         if (!(jointDefinition instanceof RevoluteJointDefinition))
            throw new UnsupportedOperationException("Only supports revolute loop closures.");

         RevoluteJointDefinition revoluteJointDefinition = (RevoluteJointDefinition) jointDefinition;
         LoopClosureDefinition loopClosureDefinition = revoluteJointDefinition.getLoopClosureDefinition();

         String name = revoluteJointDefinition.getName();
         Vector3D offsetFromParentJoint = jointDefinition.getTransformToParent().getTranslation();
         Vector3D offsetFromLinkParentJoint = loopClosureDefinition.getTransformToSuccessorParent().getTranslation();
         Vector3D axis = revoluteJointDefinition.getAxis();
         LoopClosurePinConstraintDescription loopClosurePinConstraintDescription = new LoopClosurePinConstraintDescription(name,
                                                                                                                           offsetFromParentJoint,
                                                                                                                           offsetFromLinkParentJoint,
                                                                                                                           axis);
         loopClosurePinConstraintDescription.setGains(loopClosureDefinition.getKpSoftConstraint(), loopClosureDefinition.getKdSoftConstraint());
         robotDescription.getJointDescription(revoluteJointDefinition.getParentJoint().getName()).addConstraint(loopClosurePinConstraintDescription);
         loopClosurePinConstraintDescription.setLink(robotDescription.getJointDescription(revoluteJointDefinition.getSuccessor().getParentJoint().getName())
                                                                     .getLink());
      }
   }

   public static LinkDescription toLinkDescription(RigidBodyDefinition source)
   {
      LinkDescription output = new LinkDescription(source.getName());
      output.setMass(source.getMass());
      output.setCenterOfMassOffset(source.getCenterOfMassOffset());
      output.setMomentOfInertia(source.getMomentOfInertia());
      output.setLinkGraphics(toLinkGraphicsDescription(source.getVisualDefinitions()));
      output.getCollisionMeshes().addAll(toCollisionMeshDescriptions(source.getCollisionShapeDefinitions()));
      return output;
   }

   public static JointDescription toJointDescription(JointDefinition source)
   {
      if (source instanceof SixDoFJointDefinition)
         return toFloatingJointDescription((SixDoFJointDefinition) source);
      if (source instanceof RevoluteJointDefinition)
         return toPinJointDescription((RevoluteJointDefinition) source);
      if (source instanceof PrismaticJointDefinition)
         return toSliderJointDescription((PrismaticJointDefinition) source);
      if (source instanceof InvertedFourBarJointDefinition)
         return toInvertedFourBarJointDescription((InvertedFourBarJointDefinition) source);
      return null;
   }

   public static FloatingJointDescription toFloatingJointDescription(SixDoFJointDefinition source)
   {
      FloatingJointDescription output = new FloatingJointDescription(source.getName());
      copyJointProperties(source, output);
      return output;
   }

   public static PinJointDescription toPinJointDescription(RevoluteJointDefinition source)
   {
      PinJointDescription output = new PinJointDescription(source.getName(), source.getTransformToParent().getTranslation(), source.getAxis());
      copyOneDoFJointProperties(source, output);
      return output;
   }

   public static SliderJointDescription toSliderJointDescription(PrismaticJointDefinition source)
   {
      SliderJointDescription output = new SliderJointDescription(source.getName(), source.getTransformToParent().getTranslation(), source.getAxis());
      copyOneDoFJointProperties(source, output);
      return output;
   }

   public static InvertedFourBarJointDescription toInvertedFourBarJointDescription(InvertedFourBarJointDefinition source)
   {
      Map<String, JointDefinition> nameToJointDefinitionMap = Arrays.stream(source.getFourBarJoints())
                                                                    .collect(Collectors.toMap(JointDefinition::getName, Function.identity()));

      InvertedFourBarJointDescription output = new InvertedFourBarJointDescription(source.getName());

      RevoluteJointDefinition loopClosureJointDefinition = Arrays.stream(source.getFourBarJoints()).filter(j -> j.isLoopClosure()).findFirst().get();
      LoopClosureDefinition loopClosureInfo = loopClosureJointDefinition.getLoopClosureDefinition();
      MutableObject<LinkDescription> loopClosureLink = new MutableObject<>();

      output.setFourBarJoints(Arrays.stream(source.getFourBarJoints()).filter(j -> !j.isLoopClosure()).map(j ->
      {
         PinJointDescription jointDescription = toPinJointDescription(j);
         LinkDescription linkDescription = toLinkDescription(j.getSuccessor());
         if (linkDescription.getName().equals(loopClosureJointDefinition.getSuccessor().getName()))
            loopClosureLink.setValue(linkDescription);
         jointDescription.setLink(linkDescription);
         return jointDescription;
      }).toArray(PinJointDescription[]::new));

      String name = loopClosureJointDefinition.getName();
      Vector3D offsetFromParentJoint = loopClosureJointDefinition.getTransformToParent().getTranslation();
      Vector3D offsetFromLinkParentJoint = loopClosureInfo.getTransformToSuccessorParent().getTranslation();
      Vector3D axis = loopClosureJointDefinition.getAxis();
      LoopClosurePinConstraintDescription fourBarClosure = new LoopClosurePinConstraintDescription(name,
                                                                                                   offsetFromParentJoint,
                                                                                                   offsetFromLinkParentJoint,
                                                                                                   axis);
      fourBarClosure.setGains(loopClosureInfo.getKpSoftConstraint(), loopClosureInfo.getKdSoftConstraint());
      fourBarClosure.setLink(loopClosureLink.getValue());
      output.setFourBarClosure(fourBarClosure);

      Map<String, JointDescription> nameToJointDescriptionMap = Arrays.stream(output.getFourBarJoints())
                                                                      .collect(Collectors.toMap(JointDescription::getName, Function.identity()));

      // Connect joints
      for (JointDescription jointDescription : output.getFourBarJoints())
      {
         String parentJointName = nameToJointDefinitionMap.get(jointDescription.getName()).getParentJoint().getName();
         if (!parentJointName.equals(source.getParentJoint().getName())) // Can't connect these yet
            nameToJointDescriptionMap.get(parentJointName).addJoint(jointDescription);
      }

      { // Loop closure
         String parentJointName = loopClosureJointDefinition.getParentJoint().getName();
         nameToJointDescriptionMap.get(parentJointName).addConstraint(fourBarClosure);
      }

      return output;
   }

   private static void copyOneDoFJointProperties(OneDoFJointDefinition source, OneDoFJointDescription destination)
   {
      copyJointProperties(source, destination);
      destination.setLimitStops(source.getPositionLowerLimit(), source.getPositionUpperLimit(), source.getKpSoftLimitStop(), source.getKdSoftLimitStop());
      destination.setVelocityLimits(source.getVelocityUpperLimit(), Double.NaN); // TODO Should maybe add this parameter to the definition
      destination.setEffortLimit(source.getEffortUpperLimit());
      destination.setStiction(source.getStiction());
      destination.setDamping(source.getDamping());
   }

   private static void copyJointProperties(JointDefinition source, JointDescription destination)
   {
      if (!source.getTransformToParent().getRotation().isZeroOrientation())
         LogTools.warn("Ignoring non-zero rotation for transform of joint: {}.", source.getName());
      destination.setOffsetFromParentJoint(source.getTransformToParent().getTranslation());
      source.getSensorDefinitions(IMUSensorDefinition.class).forEach(imu -> destination.addIMUSensor(toIMUSensorDescription(imu)));
      source.getSensorDefinitions(WrenchSensorDefinition.class).forEach(forceSensor -> destination.addForceSensor(toForceSensorDescription(forceSensor)));
      source.getSensorDefinitions(LidarSensorDefinition.class).forEach(lidar -> destination.addLidarSensor(toLidarSensorDescription(lidar)));
      source.getSensorDefinitions(CameraSensorDefinition.class).forEach(camera -> destination.addCameraSensor(toCameraSensorDescription(camera)));

      source.getKinematicPointDefinitions().forEach(kp -> destination.addKinematicPoint(toKinematicPointDescription(kp)));
      source.getExternalWrenchPointDefinitions().forEach(efp -> destination.addExternalForcePoint(toExternalForcePointDescription(efp)));
      source.getGroundContactPointDefinitions().forEach(gcp -> destination.addGroundContactPoint(toGroundContactPointDescription(gcp)));

      // TODO This should probably also live in the definition
      destination.setIsDynamic(true);
   }

   public static IMUSensorDescription toIMUSensorDescription(IMUSensorDefinition source)
   {
      IMUSensorDescription output = new IMUSensorDescription(source.getName(), new RigidBodyTransform(source.getTransformToJoint()));
      output.setAccelerationNoiseMean(source.getAccelerationNoiseMean());
      output.setAccelerationNoiseStandardDeviation(source.getAccelerationNoiseStandardDeviation());
      output.setAccelerationBiasMean(source.getAccelerationBiasMean());
      output.setAccelerationBiasStandardDeviation(source.getAccelerationBiasStandardDeviation());

      output.setAngularVelocityNoiseMean(source.getAngularVelocityNoiseMean());
      output.setAngularVelocityNoiseStandardDeviation(source.getAngularVelocityNoiseStandardDeviation());
      output.setAngularVelocityBiasMean(source.getAngularVelocityBiasMean());
      output.setAngularVelocityBiasStandardDeviation(source.getAngularVelocityBiasStandardDeviation());
      return output;
   }

   public static ForceSensorDescription toForceSensorDescription(WrenchSensorDefinition source)
   {
      ForceSensorDescription output = new ForceSensorDescription(source.getName(), new RigidBodyTransform(source.getTransformToJoint()));
      output.setUseGroundContactPoints(true);
      output.setUseShapeCollision(false);
      return output;
   }

   public static LidarSensorDescription toLidarSensorDescription(LidarSensorDefinition source)
   {
      LidarSensorDescription output = new LidarSensorDescription(source.getName(), source.getTransformToJoint());
      output.setSweepYawMin(source.getSweepYawMin());
      output.setSweepYawMax(source.getSweepYawMax());
      output.setHeightPitchMin(source.getHeightPitchMin());
      output.setHeightPitchMax(source.getHeightPitchMax());
      output.setMinRange(source.getMinRange());
      output.setMaxRange(source.getMaxRange());
      output.setPointsPerSweep(source.getPointsPerSweep());
      output.setScanHeight(source.getScanHeight());
      return output;
   }

   public static CameraSensorDescription toCameraSensorDescription(CameraSensorDefinition source)
   {
      CameraSensorDescription output = new CameraSensorDescription(source.getName(), source.getTransformToJoint());
      // TODO Check if the transform has to be modified as in toCameraSensorDefinition(...)
      output.setFieldOfView(source.getFieldOfView());
      output.setClipNear(source.getClipNear());
      output.setClipFar(source.getClipFar());
      output.setImageWidth(source.getImageWidth());
      output.setImageHeight(source.getImageHeight());
      return output;
   }

   public static KinematicPointDescription toKinematicPointDescription(KinematicPointDefinition source)
   {
      return new KinematicPointDescription(source.getName(), source.getTransformToParent().getTranslation());
   }

   public static ExternalForcePointDescription toExternalForcePointDescription(ExternalWrenchPointDefinition source)
   {
      return new ExternalForcePointDescription(source.getName(), source.getTransformToParent().getTranslation());
   }

   public static GroundContactPointDescription toGroundContactPointDescription(GroundContactPointDefinition source)
   {
      return new GroundContactPointDescription(source.getName(), new Vector3D(source.getTransformToParent().getTranslation()), source.getGroupIdentifier());
   }

   public static LinkGraphicsDescription toLinkGraphicsDescription(Collection<? extends VisualDefinition> source)
   {
      LinkGraphicsDescription output = new LinkGraphicsDescription();
      output.combine(toGraphics3DObject(source));
      return output;
   }

   public static Graphics3DObject toGraphics3DObject(Collection<? extends VisualDefinition> source)
   {
      Graphics3DObject output = new Graphics3DObject();
      for (VisualDefinition visualDefinition : source)
         output.combine(toGraphics3DObject(visualDefinition));
      return output;
   }

   public static Graphics3DObject toGraphics3DObject(VisualDefinition source)
   {
      if (source == null)
         return null;

      Graphics3DObject output = new Graphics3DObject();
      AffineTransformDefinition originPose = source.getOriginPose();

      if (originPose.hasTranslation())
      {
         output.translate(originPose.getTranslation());
      }

      if (originPose.hasLinearTransform())
      {
         LinearTransform3D linearTransform = originPose.getLinearTransform();

         if (linearTransform.isRotationMatrix())
         {
            output.rotate(linearTransform.getAsQuaternion());
         }
         else
         {
            if (!linearTransform.getPreScaleQuaternion().isZeroOrientation())
               output.rotate(linearTransform.getPreScaleQuaternion());
            output.scale(linearTransform.getScaleVector());
            if (!linearTransform.getPostScaleQuaternion().isZeroOrientation())
               output.rotate(linearTransform.getPostScaleQuaternion());
         }
      }

      List<Graphics3DPrimitiveInstruction> instructions = toGraphics3DPrimitiveInstruction(source.getGeometryDefinition());
      if (instructions == null || instructions.isEmpty())
         return null;

      for (Graphics3DPrimitiveInstruction instruction : instructions)
      {
         if (instruction instanceof Graphics3DInstruction)
            ((Graphics3DInstruction) instruction).setAppearance(toAppearanceDefinition(source.getMaterialDefinition()));
         output.addInstruction(instruction);
      }
      return output;
   }

   public static List<Graphics3DPrimitiveInstruction> toGraphics3DPrimitiveInstruction(GeometryDefinition source)
   {
      if (source == null)
         return null;

      if (source instanceof ArcTorus3DDefinition)
      {
         ArcTorus3DDefinition arcTorus = (ArcTorus3DDefinition) source;
         return Collections.singletonList(new ArcTorusGraphics3DInstruction(arcTorus.getStartAngle(),
                                                                            arcTorus.getEndAngle(),
                                                                            arcTorus.getMajorRadius(),
                                                                            arcTorus.getMinorRadius(),
                                                                            arcTorus.getResolution()));
      }
      else if (source instanceof Box3DDefinition)
      {
         Box3DDefinition box = (Box3DDefinition) source;
         return Collections.singletonList(new CubeGraphics3DInstruction(box.getSizeX(), box.getSizeY(), box.getSizeZ(), box.isCentered()));
      }
      else if (source instanceof Capsule3DDefinition)
      {
         Capsule3DDefinition capsule = (Capsule3DDefinition) source;
         return Collections.singletonList(new CapsuleGraphics3DInstruction(capsule.getLength(),
                                                                           capsule.getRadiusX(),
                                                                           capsule.getRadiusY(),
                                                                           capsule.getRadiusZ(),
                                                                           capsule.getResolution()));
      }
      else if (source instanceof Cone3DDefinition)
      {
         Cone3DDefinition cone = (Cone3DDefinition) source;
         return Collections.singletonList(new ConeGraphics3DInstruction(cone.getHeight(), cone.getRadius(), cone.getResolution()));
      }
      else if (source instanceof ConvexPolytope3DDefinition)
      {
         return null; // TODO Not sure here
      }
      else if (source instanceof Cylinder3DDefinition)
      {
         Cylinder3DDefinition cylinder = (Cylinder3DDefinition) source; // FIXME Handle the offset along the cylinder's axis.
         return Collections.singletonList(new CylinderGraphics3DInstruction(cylinder.getRadius(), cylinder.getLength(), cylinder.getResolution()));
      }
      else if (source instanceof Ellipsoid3DDefinition)
      {
         Ellipsoid3DDefinition ellipsoid = (Ellipsoid3DDefinition) source;
         return Collections.singletonList(new EllipsoidGraphics3DInstruction(ellipsoid.getRadiusX(),
                                                                             ellipsoid.getRadiusY(),
                                                                             ellipsoid.getRadiusZ(),
                                                                             ellipsoid.getResolution()));
      }
      else if (source instanceof ExtrudedPolygon2DDefinition)
      {
         ExtrudedPolygon2DDefinition polygon = (ExtrudedPolygon2DDefinition) source; // FIXME handle the case that bottom-z is not 0
         return Collections.singletonList(new ExtrudedPolygonGraphics3DInstruction(polygon.getPolygonVertices(), polygon.getTopZ() - polygon.getBottomZ()));
      }
      else if (source instanceof ExtrusionDefinition)
      {
         return null; // FIXME implement me
      }
      else if (source instanceof HemiEllipsoid3DDefinition)
      {
         HemiEllipsoid3DDefinition hemiEllipsoid = (HemiEllipsoid3DDefinition) source;
         return Collections.singletonList(new HemiEllipsoidGraphics3DInstruction(hemiEllipsoid.getRadiusX(),
                                                                                 hemiEllipsoid.getRadiusY(),
                                                                                 hemiEllipsoid.getRadiusZ(),
                                                                                 hemiEllipsoid.getResolution()));
      }
      else if (source instanceof ModelFileGeometryDefinition)
      {
         ModelFileGeometryDefinition model = (ModelFileGeometryDefinition) source;
         List<Graphics3DPrimitiveInstruction> output = new ArrayList<>();
         if (model.getScale() != null)
            output.add(new Graphics3DScaleInstruction(model.getScale()));
         if (model.getSubmeshes() == null || model.getSubmeshes().isEmpty())
            output.add(new Graphics3DAddModelFileInstruction(model.getFileName(), null, model.getResourceDirectories(), model.getResourceClassLoader()));
         else
            output.add(new Graphics3DAddModelFileInstruction(model.getFileName(),
                                                             model.getSubmeshes().get(0).getName(),
                                                             model.getSubmeshes().get(0).getCenter(),
                                                             null,
                                                             model.getResourceDirectories(),
                                                             model.getResourceClassLoader()));
         return output;
      }
      else if (source instanceof PyramidBox3DDefinition)
      {
         PyramidBox3DDefinition pyramidBox = (PyramidBox3DDefinition) source;
         return Collections.singletonList(new PyramidCubeGraphics3DInstruction(pyramidBox.getBoxSizeX(),
                                                                               pyramidBox.getBoxSizeY(),
                                                                               pyramidBox.getBoxSizeZ(),
                                                                               pyramidBox.getPyramidHeight()));
      }
      else if (source instanceof Ramp3DDefinition)
      {
         Ramp3DDefinition ramp = (Ramp3DDefinition) source; // FIXME The origin might not be the same.
         return Collections.singletonList(new WedgeGraphics3DInstruction(ramp.getSizeX(), ramp.getSizeY(), ramp.getSizeZ()));
      }
      else if (source instanceof Sphere3DDefinition)
      {
         Sphere3DDefinition sphere = (Sphere3DDefinition) source;
         return Collections.singletonList(new SphereGraphics3DInstruction(sphere.getRadius(), sphere.getResolution()));
      }
      else if (source instanceof Torus3DDefinition)
      {
         Torus3DDefinition torus = (Torus3DDefinition) source;
         return Collections.singletonList(new ArcTorusGraphics3DInstruction(0,
                                                                            2.0 * Math.PI,
                                                                            torus.getMajorRadius(),
                                                                            torus.getMinorRadius(),
                                                                            torus.getResolution()));
      }
      else if (source instanceof TriangleMesh3DDefinition)
      {
         TriangleMesh3DDefinition mesh = (TriangleMesh3DDefinition) source;
         return Collections.singletonList(new Graphics3DAddMeshDataInstruction(new MeshDataHolder(mesh.getVertices(),
                                                                                                  mesh.getTextures() == null ? null
                                                                                                        : Arrays.stream(mesh.getTextures())
                                                                                                                .map(t -> new TexCoord2f(t.getX32(),
                                                                                                                                         t.getY32()))
                                                                                                                .toArray(TexCoord2f[]::new),
                                                                                                  mesh.getTriangleIndices(),
                                                                                                  mesh.getNormals()),
                                                                               null));
      }
      else if (source instanceof TruncatedCone3DDefinition)
      {
         TruncatedCone3DDefinition cone = (TruncatedCone3DDefinition) source;
         return Collections.singletonList(new TruncatedConeGraphics3DInstruction(cone.getHeight(),
                                                                                 cone.getBaseRadiusX(),
                                                                                 cone.getBaseRadiusY(),
                                                                                 cone.getTopRadiusX(),
                                                                                 cone.getTopRadiusY(),
                                                                                 cone.getResolution()));
      }
      else
      {
         throw new IllegalArgumentException("Unsupported geometry type: " + source.getClass().getName());
      }
   }

   public static AppearanceDefinition toAppearanceDefinition(MaterialDefinition source)
   {
      if (source == null)
         return null;

      ColorDefinition diffuseColor = source.getDiffuseColor();
      ColorDefinition specularColor = source.getSpecularColor();
      ColorDefinition ambientColor = source.getAmbientColor();

      if (diffuseColor != null)
      {
         if (specularColor != null && ambientColor != null)
         {
            YoAppearanceMaterial output = new YoAppearanceMaterial();
            output.setDiffuseColor((float) diffuseColor.getRed(), (float) diffuseColor.getGreen(), (float) diffuseColor.getBlue());
            output.setSpecularColor((float) specularColor.getRed(), (float) specularColor.getGreen(), (float) specularColor.getBlue());
            output.setAmbientColor((float) ambientColor.getRed(), (float) ambientColor.getGreen(), (float) ambientColor.getBlue());
            output.setShininess((float) source.getShininess());
            return output;
         }
         else
         {
            return new YoAppearanceRGBColor(diffuseColor.getRed(), diffuseColor.getGreen(), diffuseColor.getBlue(), 1.0 - diffuseColor.getAlpha());
         }
      }
      else
      {
         TextureDefinition diffuseMap = source.getDiffuseMap();

         if (diffuseMap == null)
            return null;

         if (diffuseMap.getFilename() != null)
            return new YoAppearanceTexture(diffuseMap.getFilename());
         if (diffuseMap.getFileURL() != null)
            return new YoAppearanceTexture(diffuseMap.getFileURL().toExternalForm());
         if (diffuseMap.getImage() != null)
            return new YoAppearanceTexture(diffuseMap.getImage());
         return null;
      }
   }

   public static List<CollisionMeshDescription> toCollisionMeshDescriptions(Collection<? extends CollisionShapeDefinition> source)
   {
      return source.stream().map(RobotDefinitionTools::toCollisionMeshDescription).filter(Objects::nonNull).collect(Collectors.toList());
   }

   public static CollisionMeshDescription toCollisionMeshDescription(CollisionShapeDefinition source)
   {
      return null; // TODO implement me
   }
}
