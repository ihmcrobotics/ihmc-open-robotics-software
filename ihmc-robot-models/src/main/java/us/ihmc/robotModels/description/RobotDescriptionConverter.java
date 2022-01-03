package us.ihmc.robotModels.description;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Vector3D;
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
import us.ihmc.robotics.robotDescription.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.ExternalForcePointDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.KinematicPointDescription;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LoopClosureConstraintDescription;
import us.ihmc.robotics.robotDescription.LoopClosurePinConstraintDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SensorDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.scs2.definition.geometry.ArcTorus3DDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Capsule3DDefinition;
import us.ihmc.scs2.definition.geometry.Cone3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.Ellipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrudedPolygon2DDefinition;
import us.ihmc.scs2.definition.geometry.ExtrusionDefinition;
import us.ihmc.scs2.definition.geometry.HemiEllipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition.SubMeshDefinition;
import us.ihmc.scs2.definition.geometry.Polygon3DDefinition;
import us.ihmc.scs2.definition.geometry.PyramidBox3DDefinition;
import us.ihmc.scs2.definition.geometry.Ramp3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
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
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.TextureDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class RobotDescriptionConverter
{
   public static RobotDefinition toRobotDefinition(RobotDescription robotDescription)
   {
      RigidBodyDefinition rootBody = new RigidBodyDefinition(robotDescription.getName() + "RootBody");

      for (JointDescription rootJointToCopy : robotDescription.getRootJoints())
         createAndAddJointsRecursive(rootBody, rootJointToCopy);

      for (JointDescription rootJointToCopy : robotDescription.getRootJoints())
         createLoopClosureJointsRecursive(rootBody, rootJointToCopy);

      RobotDefinition robotDefinition = new RobotDefinition(robotDescription.getName());
      robotDefinition.setRootBodyDefinition(rootBody);
      return robotDefinition;
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

   public static MaterialDefinition toMaterialDefinition(AppearanceDefinition appearanceDefinition)
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

   public static ColorDefinition toColorDefinition(MutableColor mutableColor, double transparency)
   {
      return new ColorDefinition(mutableColor.getX(), mutableColor.getY(), mutableColor.getZ(), 1.0 - transparency);
   }

   private static void createLoopClosureJointsRecursive(RigidBodyDefinition rootBody, JointDescription jointDescription)
   {
      List<LoopClosureConstraintDescription> constraintDescriptions = jointDescription.getChildrenConstraintDescriptions();

      for (LoopClosureConstraintDescription constraintDescription : constraintDescriptions)
      {
         if (!(constraintDescription instanceof LoopClosurePinConstraintDescription))
            throw new UnsupportedOperationException("Only " + LoopClosurePinConstraintDescription.class.getSimpleName() + " constraints are supported.");

         RevoluteJointDefinition jointDefinition = new RevoluteJointDefinition(constraintDescription.getName());
         // TODO Being a little lazy on figuring out the joint axis.
         if (EuclidCoreTools.isZero(constraintDescription.getConstraintMomentSubSpace().getM00(), 1.0e-12))
            jointDefinition.setAxis(Axis3D.X);
         else if (EuclidCoreTools.isZero(constraintDescription.getConstraintMomentSubSpace().getM11(), 1.0e-12))
            jointDefinition.setAxis(Axis3D.Y);
         else if (EuclidCoreTools.isZero(constraintDescription.getConstraintMomentSubSpace().getM22(), 1.0e-12))
            jointDefinition.setAxis(Axis3D.Z);
         else
            throw new IllegalStateException("Unable to retrieve the joint axis.");

         jointDefinition.getTransformToParent().getTranslation().set(constraintDescription.getOffsetFromParentJoint());
         LoopClosureDefinition loopClosureDefinition = new LoopClosureDefinition();
         loopClosureDefinition.setOffsetFromSuccessorParent(constraintDescription.getOffsetFromLinkParentJoint());
         loopClosureDefinition.setKpSoftConstraint(new Vector3D(constraintDescription.getProportionalGains()));
         loopClosureDefinition.setKdSoftConstraint(new Vector3D(constraintDescription.getDerivativeGains()));
         jointDefinition.setLoopClosureDefinition(loopClosureDefinition);

         JointDefinition parentJoint = RobotDefinition.findJointDefinition(rootBody, constraintDescription.getParentJoint().getName());
         parentJoint.getSuccessor().addChildJoint(jointDefinition);
         RigidBodyDefinition successor = RobotDefinition.findRigidBodyDefinition(rootBody, constraintDescription.getLink().getName());
         jointDefinition.setLoopClosureSuccessor(successor);
      }

      for (JointDescription childJointDescription : jointDescription.getChildrenJoints())
      {
         createLoopClosureJointsRecursive(rootBody, childJointDescription);
      }
   }
}
