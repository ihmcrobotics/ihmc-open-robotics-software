package us.ihmc.avatar.factory;

import java.io.InputStream;
import java.util.Collection;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Predicate;

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
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.color.MutableColor;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.robotModels.description.RobotDefinitionConverter;
import us.ihmc.robotModels.description.RobotDescriptionConverter;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPBox3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCapsule3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPConvexPolytope3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCylinder3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPRamp3DReadOnly;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.GraphicsObjectsHolder;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.ConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Point3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCapsule3DDefinition;
import us.ihmc.scs2.definition.geometry.STPConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.STPRamp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.scs2.definition.state.interfaces.SixDoFJointStateBasics;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;

public class RobotDefinitionTools
{
   public static void setRobotInitialState(RobotDefinition definition, Robot robot)
   {
      definition.forEachJointDefinition(jointDefinition ->
      {
         if (jointDefinition.getInitialJointState() == null)
            return;
         Joint joint = robot.getJoint(jointDefinition.getName());
         if (joint == null)
         {
            LogTools.error("Could not find joint {} during initialization.", jointDefinition.getName());
            return;
         }

         if (joint instanceof FloatingJoint)
         {
            SixDoFJointStateBasics initialState = (SixDoFJointStateBasics) jointDefinition.getInitialJointState();
            FloatingJoint floatingJoint = (FloatingJoint) joint;
            if (initialState.hasOutputFor(JointStateType.CONFIGURATION))
            {
               floatingJoint.setPosition(initialState.getPosition());
               floatingJoint.setOrientation(initialState.getOrientation());
            }
            if (initialState.hasOutputFor(JointStateType.VELOCITY))
            {
               floatingJoint.setAngularVelocityInBody(initialState.getLinearVelocity());
               Vector3D linearVelocity = new Vector3D();
               floatingJoint.getOrientation().transform(initialState.getLinearVelocity(), linearVelocity);
               floatingJoint.setVelocity(linearVelocity);
            }
            if (initialState.hasOutputFor(JointStateType.ACCELERATION))
               LogTools.warn("Acceleration is not supported.");
            if (initialState.hasOutputFor(JointStateType.EFFORT))
               LogTools.warn("Effort is not supported.");
         }
         else if (joint instanceof OneDegreeOfFreedomJoint)
         {
            OneDoFJointStateBasics initialState = (OneDoFJointStateBasics) jointDefinition.getInitialJointState();
            OneDegreeOfFreedomJoint oneDoFJoint = (OneDegreeOfFreedomJoint) joint;
            if (initialState.hasOutputFor(JointStateType.CONFIGURATION))
               oneDoFJoint.setQ(initialState.getConfiguration());
            if (initialState.hasOutputFor(JointStateType.VELOCITY))
               oneDoFJoint.setQ(initialState.getVelocity());
            if (initialState.hasOutputFor(JointStateType.ACCELERATION))
               oneDoFJoint.setQ(initialState.getAcceleration());
            if (initialState.hasOutputFor(JointStateType.EFFORT))
               oneDoFJoint.setQ(initialState.getEffort());
         }
         else
         {
            LogTools.warn("Joint type not supported for initialization: {} ", joint.getClass().getSimpleName());
         }
      });
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
      if (source.getRigidBody() != null)
         output.setName(source.getRigidBody().getName());
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
         pose.set(stpRamp3D.getPose());
         // So the origin is at the center of the bottom face
         pose.appendTranslation(0.5 * stpRamp3D.getSizeX(), 0.0, 0.0);
      }
      else if (shape instanceof FrameRamp3DReadOnly)
      {
         FrameRamp3DReadOnly ramp3D = (FrameRamp3DReadOnly) shape;
         geometry = new Ramp3DDefinition(ramp3D.getSize());
         pose.set(ramp3D.getPose());
         // So the origin is at the center of the bottom face
         pose.appendTranslation(0.5 * ramp3D.getSizeX(), 0.0, 0.0);
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

   // --------------------------------------------------------
   // RobotDefinitionLoader redirections:
   // --------------------------------------------------------

   public static RobotDefinition loadURDFModel(InputStream stream,
                                               Collection<String> resourceDirectories,
                                               ClassLoader classLoader,
                                               String modelName,
                                               ContactPointDefinitionHolder contactPointDefinitionHolder,
                                               JointNameMap<?> jointNameMap,
                                               boolean removeCollisionMeshes)
   {
      return RobotDefinitionLoader.loadURDFModel(stream,
                                                 resourceDirectories,
                                                 classLoader,
                                                 modelName,
                                                 contactPointDefinitionHolder,
                                                 jointNameMap,
                                                 removeCollisionMeshes);
   }

   public static RobotDefinition loadSDFModel(InputStream stream,
                                              Collection<String> resourceDirectories,
                                              ClassLoader classLoader,
                                              String modelName,
                                              ContactPointDefinitionHolder contactPointDefinitionHolder,
                                              JointNameMap<?> jointNameMap,
                                              boolean removeCollisionMeshes)
   {
      return RobotDefinitionLoader.loadSDFModel(stream,
                                                resourceDirectories,
                                                classLoader,
                                                modelName,
                                                contactPointDefinitionHolder,
                                                jointNameMap,
                                                removeCollisionMeshes);
   }

   public static void setDefaultMaterial(RobotDefinition robotDefinition)
   {
      RobotDefinitionLoader.setDefaultMaterial(robotDefinition);
   }

   public static void setDefaultMaterial(RobotDefinition robotDefinition, MaterialDefinition defaultMaterial)
   {
      RobotDefinitionLoader.setDefaultMaterial(robotDefinition, defaultMaterial);
   }

   public static void removeCollisionShapeDefinitions(RobotDefinition robotDefinition)
   {
      RobotDefinitionLoader.removeCollisionShapeDefinitions(robotDefinition);
   }

   public static void setRobotDefinitionMaterial(RobotDefinition robotDefinition, MaterialDefinition materialDefinition)
   {
      RobotDefinitionLoader.setRobotDefinitionMaterial(robotDefinition, materialDefinition);
   }

   public static void setRobotDefinitionTransparency(RobotDefinition robotDefinition, double transparency)
   {
      RobotDefinitionLoader.setRobotDefinitionTransparency(robotDefinition, transparency);
   }

   public static void adjustJointLimitStops(RobotDefinition robotDefinition, JointNameMap<?> jointNameMap)
   {
      RobotDefinitionLoader.adjustJointLimitStops(robotDefinition, jointNameMap);
   }

   public static void adjustRigidBodyInterias(RobotDefinition robotDefinition)
   {
      RobotDefinitionLoader.adjustRigidBodyInterias(robotDefinition);
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder)
   {
      RobotDefinitionLoader.addGroundContactPoints(robotDefinition, contactPointHolder);
   }

   public static void addGroundContactPoints(RobotDefinition robotDefinition, ContactPointDefinitionHolder contactPointHolder, boolean addVisualization)
   {
      RobotDefinitionLoader.addGroundContactPoints(robotDefinition, contactPointHolder, addVisualization);
   }

   public static void scaleRobotDefinition(RobotDefinition definition, double modelScale, double massScalePower, Predicate<JointDefinition> jointFilter)
   {
      RobotDefinitionLoader.scaleRobotDefinition(definition, modelScale, massScalePower, jointFilter);
   }

   public static void scaleRigidBodyDefinitionRecursive(RigidBodyDefinition definition,
                                                        double modelScale,
                                                        double massScalePower,
                                                        Predicate<JointDefinition> jointFilter,
                                                        boolean scaleInertia)
   {
      RobotDefinitionLoader.scaleRigidBodyDefinitionRecursive(definition, modelScale, massScalePower, jointFilter, scaleInertia);
   }

   public static void scaleJointDefinition(JointDefinition definition, double modelScale, double massScalePower)
   {
      RobotDefinitionLoader.scaleJointDefinition(definition, modelScale, massScalePower);
   }

   public static void scaleRigidBodyDefinition(RigidBodyDefinition definition, double modelScale, double massScalePower, boolean scaleInertia)
   {
      RobotDefinitionLoader.scaleRigidBodyDefinition(definition, modelScale, massScalePower, scaleInertia);
   }

   public static Consumer<RobotDefinition> jointLimitRemover()
   {
      return RobotDefinitionLoader.jointLimitRemover();
   }

   public static Consumer<RobotDefinition> jointLimitRemover(String nameFilter)
   {
      return RobotDefinitionLoader.jointLimitRemover(nameFilter);
   }

   public static Consumer<RobotDefinition> jointLimitMutator(String nameFilter, double lowerLimit, double upperLimit)
   {
      return RobotDefinitionLoader.jointLimitMutator(nameFilter, lowerLimit, upperLimit);
   }

   // --------------------------------------------------
   // RobotDefinitionConverter redirections:
   // --------------------------------------------------
   public static GraphicsObjectsHolder toGraphicsObjectsHolder(RobotDefinition robotDefinition)
   {
      return RobotDefinitionConverter.toGraphicsObjectsHolder(robotDefinition);
   }

   public static RobotDescription toRobotDescription(RobotDefinition robotDefinition)
   {
      return RobotDefinitionConverter.toRobotDescription(robotDefinition);
   }

   public static Graphics3DObject toGraphics3DObject(Collection<? extends VisualDefinition> source)
   {
      return RobotDefinitionConverter.toGraphics3DObject(source);
   }

   public static Graphics3DObject toGraphics3DObject(VisualDefinition source)
   {
      return RobotDefinitionConverter.toGraphics3DObject(source);
   }

   public static List<Graphics3DPrimitiveInstruction> toGraphics3DPrimitiveInstruction(GeometryDefinition source)
   {
      return RobotDefinitionConverter.toGraphics3DPrimitiveInstruction(source);
   }

   public static AppearanceDefinition toAppearanceDefinition(MaterialDefinition source)
   {
      return RobotDefinitionConverter.toAppearanceDefinition(source);
   }

   public static List<CollisionMeshDescription> toCollisionMeshDescriptions(Collection<? extends CollisionShapeDefinition> source)
   {
      return RobotDefinitionConverter.toCollisionMeshDescriptions(source);
   }

   public static CollisionMeshDescription toCollisionMeshDescription(CollisionShapeDefinition source)
   {
      return RobotDefinitionConverter.toCollisionMeshDescription(source);
   }

   // --------------------------------------------------
   // RobotDescriptionConverter redirections:
   // --------------------------------------------------

   public static RobotDefinition toRobotDefinition(RobotDescription robotDescription)
   {
      return RobotDescriptionConverter.toRobotDefinition(robotDescription);
   }

   public static List<VisualDefinition> toVisualDefinitions(Graphics3DObject graphics3DObject)
   {
      return RobotDescriptionConverter.toVisualDefinitions(graphics3DObject);
   }

   public static MaterialDefinition toMaterialDefinition(AppearanceDefinition appearanceDefinition)
   {
      return RobotDescriptionConverter.toMaterialDefinition(appearanceDefinition);
   }

   public static ColorDefinition toColorDefinition(MutableColor mutableColor, double transparency)
   {
      return RobotDescriptionConverter.toColorDefinition(mutableColor, transparency);
   }
}
