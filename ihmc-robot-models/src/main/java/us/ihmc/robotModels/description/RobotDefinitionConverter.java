package us.ihmc.robotModels.description;

import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.graphicsDescription.instructions.*;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DTranslateInstruction;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.scs2.definition.AffineTransformDefinition;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.*;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.util.*;
import java.util.stream.Collectors;

public class RobotDefinitionConverter
{
   public static GraphicsObjectsHolder toGraphicsObjectsHolder(RobotDefinition robotDefinition)
   {
      Map<String, Graphics3DObject> cache = new HashMap<>();
      robotDefinition.forEachJointDefinition(jointDefinition ->
                                             {
                                                if (jointDefinition instanceof CrossFourBarJointDefinition)
                                                {
                                                   CrossFourBarJointDefinition crossFourBarJointDefinition = (CrossFourBarJointDefinition) jointDefinition;
                                                   cache.put(crossFourBarJointDefinition.getJointNameA(),
                                                             toLinkGraphicsDescription(crossFourBarJointDefinition.getBodyDA().getVisualDefinitions()));
                                                   cache.put(crossFourBarJointDefinition.getJointNameB(),
                                                             toLinkGraphicsDescription(crossFourBarJointDefinition.getBodyBC().getVisualDefinitions()));
                                                }
                                             });

      return jointName ->
      {
         return cache.computeIfAbsent(jointName, name ->
         {
            JointDefinition jointDefinition = robotDefinition.getJointDefinition(jointName);
            if (jointDefinition == null)
               return null;
            else
               return toLinkGraphicsDescription(jointDefinition.getSuccessor().getVisualDefinitions());
         });
      };
   }

   public static RobotDescription toRobotDescription(RobotDefinition robotDefinition)
   {
      RobotDescription robotDescription = new RobotDescription(robotDefinition.getName());

      for (JointDefinition rootJointDefinition : robotDefinition.getRootJointDefinitions())
         createAndAddJointsRecursive(robotDescription, rootJointDefinition);

      createAndAddLoopClosureJoints(robotDescription, robotDefinition);

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
      if (source instanceof CrossFourBarJointDefinition)
         return toCrossFourBarJointDescription((CrossFourBarJointDefinition) source);
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

   public static CrossFourBarJointDescription toCrossFourBarJointDescription(CrossFourBarJointDefinition source)
   {
      CrossFourBarJointDescription output = new CrossFourBarJointDescription(source.getName(), source.getAxis());
      output.setJointNames(source.getJointNameA(), source.getJointNameB(), source.getJointNameC(), source.getJointNameD());
      output.setBodyDA(toLinkDescription(source.getBodyDA()));
      output.setBodyBC(toLinkDescription(source.getBodyBC()));
      output.setJointTransforms(source.getTransformAToPredecessor(), source.getTransformBToPredecessor(), source.getTransformDToA(), source.getTransformCToB());
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

   public static List<CollisionMeshDescription> toCollisionMeshDescriptions(Collection<? extends CollisionShapeDefinition> source)
   {
      return source.stream().map(RobotDefinitionConverter::toCollisionMeshDescription).filter(Objects::nonNull).collect(Collectors.toList());
   }

   public static CollisionMeshDescription toCollisionMeshDescription(CollisionShapeDefinition source)
   {
      return null; // TODO implement me
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////
   // TODO: The following methods are copied from us.ihmc.graphicsDescription.conversion.VisualsConversionTools
   //      and should be removed once the cylinder conversion bug is fixed in the original class.
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
            ((Graphics3DInstruction) instruction).setAppearance(VisualsConversionTools.toAppearanceDefinition(source.getMaterialDefinition()));
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
         if (cylinder.isCentered())
            return List.of(new Graphics3DTranslateInstruction(0, 0, -0.5 * cylinder.getLength()),
                           new CylinderGraphics3DInstruction(cylinder.getRadius(), cylinder.getLength(), cylinder.getResolution()));
         else
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
                                                                                                  mesh.getTextures() == null ?
                                                                                                        null :
                                                                                                        Arrays.stream(mesh.getTextures())
                                                                                                              .map(t -> new TexCoord2f(t.getX32(), t.getY32()))
                                                                                                              .toArray(TexCoord2f[]::new),
                                                                                                  mesh.getTriangleIndices(),
                                                                                                  mesh.getNormals()), null));
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
}
