package us.ihmc.exampleSimulations.beetle.planning;

import java.awt.Color;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.plotting.artifact.CircleArtifact;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootStepPlanner
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private FullRobotModel fullRobotModel;
   private HexapodReferenceFrames referenceFrames;

   private SegmentDependentList<RobotSextant, YoFrameVector> nominalOffsetsFromBodyToFeet = new SegmentDependentList<>(RobotSextant.class);
   private final CenterOfMassJacobian centerOfMassJacobian;
   private FrameVector3D centerOfMassVelocity = new FrameVector3D();
   private FrameVector3D desiredVelocityScaled = new FrameVector3D();
   private ReferenceFrame centerOfMassFrameWithOrientation;
   private ReferenceFrame bodyZUpFrame;
   private YoDouble swingTimeScalar = new YoDouble("swingTimeScalar", registry);
   private final YoFrameConvexPolygon2d stancePolygon = new YoFrameConvexPolygon2d("stancePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2d swingPolygon = new YoFrameConvexPolygon2d("swingPolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameVector footScalarFromNominalToBody;
   private final YoFrameVector feetOffsetFromBody;

   private final RigidBody pelvis;
   private final Twist twistToPack = new Twist();
   private FrameVector3D angularVelocity = new FrameVector3D();
   private Vector3D perpindicularToCenterOfMassVelocity = new Vector3D();
   private FramePoint3D centerOfTurn = new FramePoint3D();

   private final CircleArtifact turnRadiusVisual = new CircleArtifact("turnRadiusVisual", 0.0, 0.0, 0.0, false);
   private final PoseReferenceFrame bodyFrameProjectedInFuture;
   private final PoseReferenceFrame bodyFrameEndRotationProjectedInFuture;
   private final YoGraphicReferenceFrame bodyFrameProjectedInFutureViz;
   private final FramePose bodyPoseProjectedInFuture = new FramePose();
   private final CircleArtifact bodyFrameProjectedInFutureCircleArtifact = new CircleArtifact("bodyFrameProjectedInFutureArtifact", 0.0, 0.0, 0.03, false);
   private final LineArtifact bodyFrameProjectedInFutureLineArtifact = new LineArtifact("bodyFrameProjectedInFutureLineArtifact");

   public FootStepPlanner(String prefix, FullRobotModel fullRobotModel, HexapodReferenceFrames hexapodReferenceFrames,
         YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = hexapodReferenceFrames;
      pelvis = fullRobotModel.getPelvis();
      swingTimeScalar.set(1.1);

      bodyZUpFrame = hexapodReferenceFrames.getBodyZUpFrame();
      centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      centerOfMassFrameWithOrientation = referenceFrames.getCenterOfMassFrameWithBodyZUpOrientation();

      bodyFrameProjectedInFuture = new PoseReferenceFrame("bodyProjectedInFuture", bodyZUpFrame);
      bodyFrameEndRotationProjectedInFuture = new PoseReferenceFrame("bodyFrameEndRotationProjectedInFuture", bodyFrameProjectedInFuture);
      bodyFrameProjectedInFutureViz = new YoGraphicReferenceFrame(bodyFrameEndRotationProjectedInFuture, registry, 0.3);
      
      yoGraphicsListRegistry.registerYoGraphic("bodyFrameProjectedInFutureViz", bodyFrameProjectedInFutureViz);

      yoGraphicsListRegistry.registerArtifact("turnRadiusVisual", turnRadiusVisual);
      yoGraphicsListRegistry.registerArtifact("bodyFrameProjectedInFutureArtifact", bodyFrameProjectedInFutureCircleArtifact);
      bodyFrameProjectedInFutureCircleArtifact.setColor(Color.DARK_GRAY);
      bodyFrameProjectedInFutureCircleArtifact.setRecordHistory(true);

      yoGraphicsListRegistry.registerArtifact("bodyFrameProjectedInFutureLineArtifact", bodyFrameProjectedInFutureLineArtifact);
      bodyFrameProjectedInFutureLineArtifact.setColor(Color.RED);
      bodyFrameProjectedInFutureLineArtifact.setRecordHistory(true);

      footScalarFromNominalToBody = new YoFrameVector(prefix + "footScalarFromNominalToBody", bodyZUpFrame, registry);
      footScalarFromNominalToBody.set(1.0, 0.6, 1.0);

      feetOffsetFromBody = new YoFrameVector(prefix + "feetOffsetFromBody", bodyZUpFrame, registry);
      feetOffsetFromBody.set(-0.002, 0.0, 0.0);

      FramePoint3D temp = new FramePoint3D();
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
         temp.setToZero(footFrame);
         temp.changeFrame(bodyZUpFrame);
         Vector3D offsetFromBodyToFoot = new Vector3D();
         temp.get(offsetFromBodyToFoot);
         YoFrameVector yoOffset = new YoFrameVector(prefix + robotSextant.name() + "offsetFromBodyToFoot", bodyZUpFrame, registry);
         yoOffset.set(offsetFromBodyToFoot);
         if (robotSextant == RobotSextant.FRONT_LEFT || robotSextant == RobotSextant.FRONT_RIGHT)
         {
            yoOffset.setX(Math.signum(yoOffset.getX()) * 0.2);
         }
         if (robotSextant == RobotSextant.MIDDLE_LEFT || robotSextant == RobotSextant.MIDDLE_RIGHT)
         {
            yoOffset.setX(Math.signum(yoOffset.getX()) * 0.0);
         }
         if (robotSextant == RobotSextant.HIND_LEFT || robotSextant == RobotSextant.HIND_RIGHT)
         {
            yoOffset.setX(Math.signum(yoOffset.getX()) * 0.13);
         }
         nominalOffsetsFromBodyToFeet.set(robotSextant, yoOffset);
      }

      YoArtifactPolygon stancePolygonArtifact = new YoArtifactPolygon(prefix + "stancePolygonArtifact", stancePolygon, Color.blue, false);
      YoArtifactPolygon swingPolygonArtifact = new YoArtifactPolygon(prefix + "swingPolygonArtifact", swingPolygon, Color.RED, false);

      yoGraphicsListRegistry.registerArtifact(prefix + "stancePolygonArtifact", stancePolygonArtifact);
      yoGraphicsListRegistry.registerArtifact(prefix + "swingPolygonArtifact", swingPolygonArtifact);
      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(true);

      parentRegistry.addChild(registry);
   }

   FrameVector3D offsetFromBodyToFoot = new FrameVector3D();

   private final Point2D startPoint = new Point2D();
   private final FramePoint2D frameEndPoint = new FramePoint2D();
   private final FramePoint3D bodyPositionProjectedInFuture = new FramePoint3D();
   private final FrameQuaternion rotationAtEnd = new FrameQuaternion();
   
   public void getDesiredFootPosition(RobotSextant robotSextant, FrameVector3D desiredLinearVelocity, FrameVector3D desiredAngularVelocity, double swingTime, FramePoint3D framePointToPack)
   {
      YoFrameVector offsetFromBodyToFootDesired = nominalOffsetsFromBodyToFeet.get(robotSextant);
      offsetFromBodyToFootDesired.getFrameTupleIncludingFrame(offsetFromBodyToFoot);
      offsetFromBodyToFoot.scale(footScalarFromNominalToBody.getX(), footScalarFromNominalToBody.getY(), footScalarFromNominalToBody.getZ());
      
      desiredVelocityScaled.setToNaN(desiredLinearVelocity.getReferenceFrame());
      desiredVelocityScaled.setAndScale(swingTime * swingTimeScalar.getDoubleValue(), desiredLinearVelocity);
      desiredVelocityScaled.changeFrame(centerOfMassFrameWithOrientation);
      
//      if(desiredAngularVelocity.length() > 0.003)
//      {
         double radius = desiredLinearVelocity.length() / desiredAngularVelocity.getZ();
         turnRadiusVisual.setDiameter(radius * 2.0);
         perpindicularToCenterOfMassVelocity.set(-desiredLinearVelocity.getY(), desiredLinearVelocity.getX(), 0.0);
         perpindicularToCenterOfMassVelocity.normalize();
         perpindicularToCenterOfMassVelocity.scale(radius);
         
         centerOfTurn.setToZero(centerOfMassFrameWithOrientation);
         centerOfTurn.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfTurn.add(perpindicularToCenterOfMassVelocity);
         turnRadiusVisual.setPosition(centerOfTurn.getX(), centerOfTurn.getY());
         
         bodyPoseProjectedInFuture.setToZero(bodyZUpFrame);
         bodyPoseProjectedInFuture.getPositionIncludingFrame(bodyPositionProjectedInFuture);
         centerOfTurn.changeFrame(bodyZUpFrame);
         GeometryTools.yawAboutPoint(bodyPositionProjectedInFuture, centerOfTurn, desiredAngularVelocity.getZ() * swingTime, bodyPositionProjectedInFuture);
         bodyPoseProjectedInFuture.setPosition(bodyPositionProjectedInFuture);
         bodyFrameProjectedInFuture.setPoseAndUpdate(bodyPoseProjectedInFuture);
         
         rotationAtEnd.setToZero(bodyFrameProjectedInFuture);
         rotationAtEnd.setYawPitchRoll(desiredAngularVelocity.getZ() * swingTime, 0.0, 0.0);
         bodyFrameEndRotationProjectedInFuture.setOrientationAndUpdate(rotationAtEnd);

         bodyFrameProjectedInFutureViz.update();
         bodyPoseProjectedInFuture.changeFrame(ReferenceFrame.getWorldFrame());
         bodyFrameProjectedInFutureCircleArtifact.setPosition(bodyPoseProjectedInFuture.getX(), bodyPoseProjectedInFuture.getY());
         startPoint.set(bodyPoseProjectedInFuture.getX(), bodyPoseProjectedInFuture.getY());
         frameEndPoint.setToZero(bodyFrameEndRotationProjectedInFuture);
         frameEndPoint.setX(0.3);
         frameEndPoint.changeFrame(ReferenceFrame.getWorldFrame());

         bodyFrameProjectedInFutureLineArtifact.setPoints(startPoint, frameEndPoint.getPoint());
//      }
//      else
//      {
//         bodyPoseProjectedInFuture.setToZero(centerOfMassFrameWithOrientation);
//         bodyPoseProjectedInFuture.setPosition(desiredVelocityScaled.getVector());
//         bodyPoseProjectedInFuture.changeFrame(bodyZUpFrame);
//         bodyFrameProjectedInFuture.setPoseAndUpdate(bodyPoseProjectedInFuture);
//         rotationAtEnd.setToZero(bodyFrameProjectedInFuture);
//         bodyFrameEndRotationProjectedInFuture.setOrientationAndUpdate(rotationAtEnd);
//      }
      
      framePointToPack.setToZero(bodyFrameEndRotationProjectedInFuture);
      framePointToPack.add(offsetFromBodyToFoot.getVector());
      framePointToPack.add(feetOffsetFromBody.getFrameTuple().getVector());
      framePointToPack.changeFrame(ReferenceFrame.getWorldFrame());
      framePointToPack.setZ(0.0);
      
      //      YoFrameVector offsetFromBodyToFootDesired = nominalOffsetsFromBodyToFeet.get(robotSextant);
      //      framePointToPack.setToZero(bodyZUpFrame);
      //      offsetFromBodyToFootDesired.getFrameTupleIncludingFrame(offsetFromBodyToFoot);
      //      offsetFromBodyToFoot.scale(footScalarFromNominalToBody.getX(), footScalarFromNominalToBody.getY(), footScalarFromNominalToBody.getZ());
      //      framePointToPack.add(offsetFromBodyToFoot);
      //      framePointToPack.add(feetOffsetFromBody.getFrameTuple());
      //      framePointToPack.changeFrame(centerOfMassFrame);
      //      framePointToPack.add(centerOfMassVelocityScaled);
      //      framePointToPack.changeFrame(ReferenceFrame.getWorldFrame());
      //      framePointToPack.setZ(0.0);
   }

   public void getDesiredFootPosition(RobotSextant robotSextant, double swingTime, FramePoint3D framePointToPack)
   {
      pelvis.getBodyFixedFrame().getTwistOfFrame(twistToPack);
      twistToPack.changeFrame(ReferenceFrame.getWorldFrame());
      twistToPack.getAngularPart(angularVelocity);

      centerOfMassJacobian.compute();
      centerOfMassJacobian.getCenterOfMassVelocity(centerOfMassVelocity);
      centerOfMassVelocity.changeFrame(ReferenceFrame.getWorldFrame());

      getDesiredFootPosition(robotSextant, centerOfMassVelocity, angularVelocity, swingTime, framePointToPack);
   }

   public void drawSupportPolygon(RobotSextant[] feet)
   {
      drawSupportPolygon(feet, stancePolygon);
   }

   public void drawSwingPolygon(RobotSextant[] feet)
   {
      drawSupportPolygon(feet, swingPolygon);
   }

   FramePoint3D footPosition = new FramePoint3D();
   ConvexPolygon2D polygon = new ConvexPolygon2D();

   private void drawSupportPolygon(RobotSextant[] feet, YoFrameConvexPolygon2d yoFramePolygon)
   {
      polygon = new ConvexPolygon2D();
      for (RobotSextant robotSextant : feet)
      {
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
         footPosition.setToZero(footFrame);
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         polygon.addVertex(footPosition.getX(), footPosition.getY());

      }
      polygon.update();
      yoFramePolygon.setConvexPolygon2d(polygon);
   }
}
