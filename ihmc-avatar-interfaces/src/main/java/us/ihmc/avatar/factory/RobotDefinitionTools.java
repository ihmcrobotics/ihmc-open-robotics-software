package us.ihmc.avatar.factory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
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
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPBox3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCapsule3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPConvexPolytope3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPCylinder3DReadOnly;
import us.ihmc.robotics.geometry.shapes.interfaces.FrameSTPRamp3DReadOnly;
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
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCapsule3DDefinition;
import us.ihmc.scs2.definition.geometry.STPConvexPolytope3DDefinition;
import us.ihmc.scs2.definition.geometry.STPCylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.STPRamp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.geometry.TriangleMesh3DDefinition;
import us.ihmc.scs2.definition.geometry.TruncatedCone3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
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
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
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

      if (shape instanceof FrameSTPBox3DReadOnly)
      {
         FrameSTPBox3DReadOnly stpBox3D = (FrameSTPBox3DReadOnly) shape;
         STPBox3DDefinition stpGeometry = new STPBox3DDefinition(stpBox3D.getSize());
         stpGeometry.setMargins(stpBox3D.getMinimumMargin(), stpBox3D.getMaximumMargin());
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
                  visualDefinition.setGeometryDefinition(new ExtrudedPolygon2DDefinition(extrusion.getPolygonPoints().stream().map(Point2D::new)
                                                                                                  .collect(Collectors.toList()),
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
                  visualDefinition.setGeometryDefinition(new Polygon3DDefinition(polygon.getPolygonPoints().stream().map(Point3D::new)
                                                                                        .collect(Collectors.toList()),
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
}
