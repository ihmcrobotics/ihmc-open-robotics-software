package us.ihmc.rdx.simulation.scs2;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.*;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DIdentityInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DRotateInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DTranslateInstruction;
import us.ihmc.robotModels.description.CrossFourBarJointDescription;
import us.ihmc.robotics.robotDescription.*;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.*;
import us.ihmc.scs2.definition.robot.*;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.util.ArrayList;
import java.util.List;

public class SCS2Tools
{
   public static RobotDefinition convertRobotDescriptionToRobotDefinition(RobotDescription robotDescription)
   {
      RobotDefinition robotDefinition = new RobotDefinition(robotDescription.getName());
      JointDescription rootJointDescription = robotDescription.getRootJoints().get(0);
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("elevator");
      convertJointDescriptionChildrenToRigidBodyDefinition(rootJointDescription, rootBodyDefinition);
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);

      return robotDefinition;
   }

   public static void convertJointDescriptionChildrenToRigidBodyDefinition(JointDescription jointDescription, RigidBodyDefinition rigidBodyDefinition)
   {
      JointDefinition jointDefinition = null;
      if (jointDescription instanceof FloatingJointDescription)
      {
         jointDefinition = new SixDoFJointDefinition(jointDescription.getName());
      }
      else if (jointDescription instanceof FloatingPlanarJointDescription)
      {

      }
      else if (jointDescription instanceof BallAndSocketJointDescription)
      {

      }
      else if (jointDescription instanceof OneDoFJointDescription)
      {
         if (jointDescription instanceof CrossFourBarJointDescription)
         {

         }
         else if (jointDescription instanceof PinJointDescription)
         {
            PinJointDescription pinJointDescription = (PinJointDescription) jointDescription;
            Vector3DReadOnly jointAxis = pinJointDescription.getJointAxis();
            double effortLimit = pinJointDescription.getEffortLimit();
            double damping = pinJointDescription.getDamping();
            double lowerLimit = pinJointDescription.getLowerLimit();
            double upperLimit = pinJointDescription.getUpperLimit();
            double stiction = pinJointDescription.getStiction();
            double velocityDamping = pinJointDescription.getVelocityDamping(); // TODO: ??
            double velocityLimit = pinJointDescription.getVelocityLimit();
            RevoluteJointDefinition revoluteJointDefinition = new RevoluteJointDefinition(pinJointDescription.getName());
            revoluteJointDefinition.setAxis(jointAxis);
            revoluteJointDefinition.setEffortLimits(effortLimit);
            revoluteJointDefinition.setDamping(damping);
            revoluteJointDefinition.setPositionLimits(lowerLimit, upperLimit);
            revoluteJointDefinition.setStiction(stiction);
            revoluteJointDefinition.setVelocityLimits(velocityLimit);
            jointDefinition = revoluteJointDefinition;
         }
         else if (jointDescription instanceof SliderJointDescription)
         {

         }
      }

      Vector3DReadOnly offsetFromParentJoint = jointDescription.getOffsetFromParentJoint();
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.getTranslation().set(offsetFromParentJoint);
      jointDefinition.setTransformToParent(transformToParent);

      RigidBodyDefinition linkRigidBodyDefinition = convertLinkDescriptionToRigidBodyDefinition(jointDescription.getLink());
      jointDefinition.setSuccessor(linkRigidBodyDefinition);
      rigidBodyDefinition.addChildJoint(jointDefinition);

      for (JointDescription childrenJoint : jointDescription.getChildrenJoints())
      {
         convertJointDescriptionChildrenToRigidBodyDefinition(childrenJoint, linkRigidBodyDefinition);
      }
   }

   public static RigidBodyDefinition convertLinkDescriptionToRigidBodyDefinition(LinkDescription linkDescription)
   {
      RigidBodyDefinition rigidBodyDefinition = new RigidBodyDefinition(linkDescription.getName());

      double mass = linkDescription.getMass();
      Vector3D centerOfMassOffset = linkDescription.getCenterOfMassOffset();
      DMatrixRMaj momentOfInertia = linkDescription.getMomentOfInertia();
      rigidBodyDefinition.setMass(mass);
      rigidBodyDefinition.setCenterOfMassOffset(centerOfMassOffset);
      rigidBodyDefinition.setMomentOfInertia(new Matrix3D(momentOfInertia));

      LinkGraphicsDescription pelvisLinkGraphics = linkDescription.getLinkGraphics();
      rigidBodyDefinition.addVisualDefinitions(convertLinkGraphicsDescriptionToVisualDefinitions(pelvisLinkGraphics));

      List<CollisionMeshDescription> collisionMeshes = linkDescription.getCollisionMeshes();
      convertCollisionMeshes(rigidBodyDefinition, collisionMeshes);

      return rigidBodyDefinition;
   }

   public static List<VisualDefinition> convertLinkGraphicsDescriptionToVisualDefinitions(LinkGraphicsDescription linkGraphicsDescription)
   {
      List<Graphics3DPrimitiveInstruction> graphics3DInstructions = linkGraphicsDescription.getGraphics3DInstructions();
      List<VisualDefinition> visualDefinitions = new ArrayList<>();

      AffineTransform originPose = new AffineTransform();
      for (Graphics3DPrimitiveInstruction graphics3DPrimitiveInstruction : graphics3DInstructions)
      {
         VisualDefinition visualDefinition = new VisualDefinition();

         if (graphics3DPrimitiveInstruction instanceof Graphics3DIdentityInstruction)
         {

         }
         else if (graphics3DPrimitiveInstruction instanceof Graphics3DScaleInstruction)
         {
            Graphics3DScaleInstruction graphics3DScaleInstruction = (Graphics3DScaleInstruction) graphics3DPrimitiveInstruction;
            originPose.appendScale(graphics3DScaleInstruction.getScaleFactor());
         }
         else if (graphics3DPrimitiveInstruction instanceof Graphics3DTranslateInstruction)
         {
            Graphics3DTranslateInstruction graphics3DTranslateInstruction = (Graphics3DTranslateInstruction) graphics3DPrimitiveInstruction;
            originPose.getTranslation().add(graphics3DTranslateInstruction.getTranslation());
         }
         else if (graphics3DPrimitiveInstruction instanceof Graphics3DRotateInstruction)
         {
            Graphics3DRotateInstruction graphics3DRotateInstruction = (Graphics3DRotateInstruction) graphics3DPrimitiveInstruction;
            originPose.getLinearTransform().appendRotation(graphics3DRotateInstruction.getRotationMatrix());
         }
         visualDefinition.setOriginPose(new AffineTransform(originPose));

         if (graphics3DPrimitiveInstruction instanceof Graphics3DInstruction)
         {
            Graphics3DInstruction graphics3DInstruction = (Graphics3DInstruction) graphics3DPrimitiveInstruction;
            AppearanceDefinition appearance = graphics3DInstruction.getAppearance();
            if (appearance != null)
            {
               ColorDefinition colorDefinition = new ColorDefinition(appearance.getColor().getX(),
                                                                     appearance.getColor().getY(),
                                                                     appearance.getColor().getZ(),
                                                                     appearance.getTransparency());
               MaterialDefinition materialDefinition = new MaterialDefinition(colorDefinition);
               visualDefinition.setMaterialDefinition(materialDefinition);
            }

            GeometryDefinition geometryDefinition = null;
            if (graphics3DInstruction instanceof Graphics3DAddMeshDataInstruction)
            {
               Graphics3DAddMeshDataInstruction graphics3DAddMeshDataInstruction = (Graphics3DAddMeshDataInstruction) graphics3DInstruction;
            }
            else if (graphics3DInstruction instanceof Graphics3DAddHeightMapInstruction)
            {
               Graphics3DAddHeightMapInstruction graphics3DAddHeightMapInstruction = (Graphics3DAddHeightMapInstruction) graphics3DInstruction;
            }
            else if (graphics3DInstruction instanceof Graphics3DAddModelFileInstruction)
            {
               Graphics3DAddModelFileInstruction graphics3DAddModelFileInstruction = (Graphics3DAddModelFileInstruction) graphics3DInstruction;
               geometryDefinition = new ModelFileGeometryDefinition(graphics3DAddModelFileInstruction.getFileName());
            }
            else if (graphics3DInstruction instanceof Graphics3DAddExtrusionInstruction)
            {
               Graphics3DAddExtrusionInstruction graphics3DAddExtrusionInstruction = (Graphics3DAddExtrusionInstruction) graphics3DInstruction;
               geometryDefinition = new ExtrusionDefinition(graphics3DAddExtrusionInstruction.getBufferedImage(),
                                                            graphics3DAddExtrusionInstruction.getHeight());
            }
            else if (graphics3DInstruction instanceof CubeGraphics3DInstruction)
            {
               CubeGraphics3DInstruction cubeGraphics3DInstruction = (CubeGraphics3DInstruction) graphics3DInstruction;
               double lengthX = cubeGraphics3DInstruction.getLength();
               double widthY = cubeGraphics3DInstruction.getWidth();
               double heightZ = cubeGraphics3DInstruction.getHeight();
               boolean centeredInTheCenter = cubeGraphics3DInstruction.getCenteredInTheCenter();
               Box3DDefinition box3DDefinition = new Box3DDefinition(lengthX, widthY, heightZ, centeredInTheCenter);
               geometryDefinition = box3DDefinition;
            }
            else if (graphics3DInstruction instanceof PolygonGraphics3DInstruction)
            {
               PolygonGraphics3DInstruction polygonGraphics3DInstruction = (PolygonGraphics3DInstruction) graphics3DInstruction;
            }
            else if (graphics3DInstruction instanceof TruncatedConeGraphics3DInstruction)
            {
               TruncatedConeGraphics3DInstruction truncatedConeGraphics3DInstruction = (TruncatedConeGraphics3DInstruction) graphics3DInstruction;
            }
            else if (graphics3DInstruction instanceof EllipsoidGraphics3DInstruction)
            {

            }
            else if (graphics3DInstruction instanceof WedgeGraphics3DInstruction)
            {

            }
            else if (graphics3DInstruction instanceof SphereGraphics3DInstruction)
            {
               SphereGraphics3DInstruction sphereGraphics3DInstruction = (SphereGraphics3DInstruction) graphics3DInstruction;
               double radius = sphereGraphics3DInstruction.getRadius();
               int resolution = sphereGraphics3DInstruction.getResolution();
               Sphere3DDefinition sphere3DDefinition = new Sphere3DDefinition(radius, resolution);
               geometryDefinition = sphere3DDefinition;
            }
            else if (graphics3DInstruction instanceof ConeGraphics3DInstruction)
            {

            }
            else if (graphics3DInstruction instanceof ArcTorusGraphics3DInstruction)
            {

            }
            else if (graphics3DInstruction instanceof HemiEllipsoidGraphics3DInstruction)
            {

            }
            else if (graphics3DInstruction instanceof ExtrudedPolygonGraphics3DInstruction)
            {

            }
            else if (graphics3DInstruction instanceof CapsuleGraphics3DInstruction)
            {

            }
            else if (graphics3DInstruction instanceof CylinderGraphics3DInstruction)
            {

            }
            else if (graphics3DInstruction instanceof PyramidCubeGraphics3DInstruction)
            {

            }
            if (geometryDefinition == null)
            {
               throw new RuntimeException("Implement " + graphics3DInstruction);
            }

            visualDefinition.setGeometryDefinition(geometryDefinition);

            visualDefinitions.add(visualDefinition);
         }
      }

      return visualDefinitions;
   }

   public static void convertCollisionMeshes(RigidBodyDefinition rigidBodyDefinition, List<CollisionMeshDescription> collisionMeshes)
   {
      for (CollisionMeshDescription collisionMesh : collisionMeshes)
      {
         ArrayList<ConvexShapeDescriptionReadOnly> convexShapeDescriptions = new ArrayList<>();
         collisionMesh.getConvexShapeDescriptions(convexShapeDescriptions);
         for (ConvexShapeDescriptionReadOnly convexShapeDescription : convexShapeDescriptions)
         {
            RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
            GeometryDefinition geometryDefinition = null;
            if (convexShapeDescription instanceof ConvexPolytopeDescriptionReadOnly)
            {

            }
            else if (convexShapeDescription instanceof CubeDescriptionReadOnly)
            {
               CubeDescriptionReadOnly cubeDescriptionReadOnly = (CubeDescriptionReadOnly) convexShapeDescription;
               double lengthX = cubeDescriptionReadOnly.getLengthX();
               double widthY = cubeDescriptionReadOnly.getWidthY();
               double heightZ = cubeDescriptionReadOnly.getHeightZ();
               cubeDescriptionReadOnly.getRigidBodyTransformToCenter(rigidBodyTransform);
               geometryDefinition = new Box3DDefinition(lengthX, widthY, heightZ);
            }
            else if (convexShapeDescription instanceof SphereDescriptionReadOnly)
            {

            }
            else if (convexShapeDescription instanceof CylinderDescriptionReadOnly)
            {

            }
            else if (convexShapeDescription instanceof CapsuleDescriptionReadOnly)
            {

            }

            CollisionShapeDefinition collisionShapeDefinition = new CollisionShapeDefinition();
            collisionShapeDefinition.setGeometryDefinition(geometryDefinition);
            collisionShapeDefinition.setOriginPose(rigidBodyTransform);
            rigidBodyDefinition.addCollisionShapeDefinition(collisionShapeDefinition);
         }
      }
   }

   public static RobotDefinition singleBodyRobot(RigidBodyDefinition rigidBodyDefinition)
   {
      RobotDefinition robotDefinition = new RobotDefinition(rigidBodyDefinition.getName() + "Robot");
      RigidBodyDefinition rootBodyDefinition = new RigidBodyDefinition("rootBody");
      robotDefinition.setRootBodyDefinition(rootBodyDefinition);
      SixDoFJointDefinition sixDoFJointDefinition = new SixDoFJointDefinition("rootJoint");
      rootBodyDefinition.addChildJoint(sixDoFJointDefinition);
      sixDoFJointDefinition.setSuccessor(rigidBodyDefinition);
      return robotDefinition;
   }

   public static void setInitialPoseOfRobot(RobotDefinition robotDefinition, Orientation3DReadOnly orientation, Tuple3DReadOnly position)
   {
      SixDoFJointState initialRootJointState = new SixDoFJointState(orientation, position);
      initialRootJointState.setVelocity(new Vector3D(), new Vector3D());
      robotDefinition.getRootJointDefinitions().get(0).setInitialJointState(initialRootJointState);
   }
}
