package us.ihmc.avatar.multiContact.pushRecovery;

import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.awt.*;
import java.util.ArrayList;

/**
 * Reduced order robot model consisting of the centroidal and contact states.
 */
public class ReducedOrderRobotModel
{
   // Plan with slower acceleration
   public static final double MAX_HAND_ACCELERATION = 6.5; // 15.0;

   // Scale down feet contact points in order to check stability conditions
   private static final double FOOT_SCALE_FACTOR = 0.6;

   /* Nominal offset in mid-feet zup frame from CoM to shoulder position, taken at default standing home pose */
   private static final double SHOULDER_COM_OFFSET_X = -0.005;
   private static final double SHOULDER_COM_OFFSET_Y = 0.242;
   private static final double SHOULDER_COM_OFFSET_Z = 0.536;

   /* Minimum and maximum allowed reachablity distance from shoulder to hand */
   public static final double REACHABILITY_RADIUS_MIN = 0.32;
   public static final double REACHABILITY_RADIUS_MAX = 0.73;

   /* Maximum inward reaching distance, to prevent too much cross-over */
   public static final double MAX_INWARD_DISTANCE = SHOULDER_COM_OFFSET_Y * 0.9;

   public static final double NOMINAL_COM_HEIGHT = 0.9;
   public static final double OMEGA = Math.sqrt(9.81 / NOMINAL_COM_HEIGHT);

   // Foot support points
   private final SegmentDependentList<RobotSide, ArrayList<Point2D>> footContactPoints;
   private final YoFrameConvexPolygon2D nominalFootPolygon;
   private final SideDependentList<YoFrameConvexPolygon2D> defaultFootPolygons = new SideDependentList<>();

   // Centroidal data
   private final YoFrameVector3D comVelocity;
   private final YoFramePoint2D capturePointPosition;
   private final YoFramePose3D centroidalPose;
   private final PoseReferenceFrame centroidalFrame = new PoseReferenceFrame("centroidalFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame midFeetZUpFrame = new PoseReferenceFrame("midFeetZUpFrame", ReferenceFrame.getWorldFrame());
   private final FramePose3D midFeetZUpPose = new FramePose3D();

   // Hand and reachability data
   private final SideDependentList<YoFramePoint3D> handPositions = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> shoulderPositions = new SideDependentList<>();

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameQuaternion tempOrientation = new FrameQuaternion();

   public ReducedOrderRobotModel(RobotContactPointParameters<RobotSide> contactPointParameters, YoRegistry registry)
   {
      this("", contactPointParameters, registry);
   }

   public ReducedOrderRobotModel(String namePrefix, RobotContactPointParameters<RobotSide> contactPointParameters, YoRegistry registry)
   {
      footContactPoints = contactPointParameters.getFootContactPoints();
      centroidalPose = new YoFramePose3D(namePrefix + "centroidalPose", ReferenceFrame.getWorldFrame(), registry);
      comVelocity = new YoFrameVector3D(namePrefix + "comVelocity", ReferenceFrame.getWorldFrame(), registry);
      capturePointPosition = new YoFramePoint2D(namePrefix + "capturePointPosition", ReferenceFrame.getWorldFrame(), registry);
      nominalFootPolygon = new YoFrameConvexPolygon2D(namePrefix + "nominalFootPolygon", ReferenceFrame.getWorldFrame(), 8, registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "_" + namePrefix;

         handPositions.put(robotSide, new YoFramePoint3D(sidePrefix + "HandPosition", ReferenceFrame.getWorldFrame(), registry));
         shoulderPositions.put(robotSide, new YoFramePoint3D(sidePrefix + "ShoulderPosition", centroidalFrame, registry));
         shoulderPositions.get(robotSide).set(SHOULDER_COM_OFFSET_X, robotSide.negateIfRightSide(SHOULDER_COM_OFFSET_Y), SHOULDER_COM_OFFSET_Z);

         defaultFootPolygons.put(robotSide, new YoFrameConvexPolygon2D(sidePrefix + "FootPolygon", ReferenceFrame.getWorldFrame(), 4, registry));
         ArrayList<Point2D> contactPoints = footContactPoints.get(robotSide);

         for (int i = 0; i < contactPoints.size(); i++)
         {
            defaultFootPolygons.get(robotSide).addVertex(contactPoints.get(i));
         }

         defaultFootPolygons.get(robotSide).update();
         defaultFootPolygons.get(robotSide).scale(FOOT_SCALE_FACTOR);
      }
   }

   public void set(ReducedOrderRobotModel other)
   {
      this.nominalFootPolygon.set(other.nominalFootPolygon);
      this.comVelocity.set(other.comVelocity);

      for (RobotSide robotSide : RobotSide.values)
      {
         handPositions.get(robotSide).set(other.handPositions.get(robotSide));
      }

      nominalFootPolygon.set(other.nominalFootPolygon);
      centroidalPose.set(other.centroidalPose);
      centroidalFrame.setPoseAndUpdate(centroidalPose);

      midFeetZUpPose.set(other.midFeetZUpPose);
      midFeetZUpFrame.setPoseAndUpdate(midFeetZUpPose);

      updateCapturePoint();
   }

   private void updateCentroidalFrame(HumanoidReferenceFrames referenceFrames)
   {
      tempPoint.setToZero(referenceFrames.getCenterOfMassFrame());
      tempOrientation.setToZero(referenceFrames.getMidFeetZUpFrame());

      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      tempOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      centroidalPose.set(tempPoint, tempOrientation);
      centroidalFrame.setPoseAndUpdate(centroidalPose);

      midFeetZUpPose.setToZero(referenceFrames.getMidFeetZUpFrame());
      midFeetZUpPose.changeFrame(ReferenceFrame.getWorldFrame());
      midFeetZUpFrame.setPoseAndUpdate(midFeetZUpPose);
   }

   private void updateSupportPolygon(SideDependentList<? extends ReferenceFrame> soleFrames)
   {
      nominalFootPolygon.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point2D> contactPoints = footContactPoints.get(robotSide);
         for (int i = 0; i < contactPoints.size(); i++)
         {
            FramePoint3D contactPoint = new FramePoint3D(soleFrames.get(robotSide), contactPoints.get(i));
            nominalFootPolygon.addVertexMatchingFrame(contactPoint);
         }
      }

      nominalFootPolygon.update();
   }

   public void updateCapturePoint()
   {
      capturePointPosition.set(comVelocity);
      capturePointPosition.scale(1.0 / OMEGA);
      capturePointPosition.add(getComPosition().getX(), getComPosition().getY());
   }

   public FixedFramePoint3DBasics getHandPosition(RobotSide robotSide)
   {
      return handPositions.get(robotSide);
   }

   public FixedFramePoint3DBasics getShoulderPosition(RobotSide robotSide)
   {
      return shoulderPositions.get(robotSide);
   }

   public FixedFramePoint3DBasics getComPosition()
   {
      return centroidalPose.getPosition();
   }

   public FixedFrameVector3DBasics getComVelocity()
   {
      return comVelocity;
   }

   public FramePose3DReadOnly getCentroidalPose()
   {
      return centroidalPose;
   }

   public ReferenceFrame getCentroidalFrame()
   {
      return centroidalFrame;
   }

   public PoseReferenceFrame getMidFeetZUpFrame()
   {
      return midFeetZUpFrame;
   }

   public FrameConvexPolygon2DReadOnly getNominalFootPolygon()
   {
      return nominalFootPolygon;
   }

   public FramePoint2DReadOnly getCapturePointPosition()
   {
      return capturePointPosition;
   }

   public void initialize(FullHumanoidRobotModel fullRobotModel, HumanoidReferenceFrames referenceFrames, FrameVector3DReadOnly comVelocity)
   {
      this.comVelocity.setMatchingFrame(comVelocity);

      for (RobotSide robotSide : RobotSide.values)
      {
         this.handPositions.get(robotSide).setFromReferenceFrame(fullRobotModel.getHandControlFrame(robotSide));
      }

      updateCentroidalFrame(referenceFrames);
      updateSupportPolygon(referenceFrames.getSoleFrames());
      updateCapturePoint();
   }

   public void moveCoMFrameToPosition(Tuple2DReadOnly comPosition)
   {
      centroidalPose.getPosition().set(comPosition);
      centroidalFrame.setPoseAndUpdate(centroidalPose);
   }

   public void moveCoMFrameToPosition(Tuple3DReadOnly comPosition)
   {
      centroidalPose.getPosition().set(comPosition);
      centroidalFrame.setPoseAndUpdate(centroidalPose);
   }

   public boolean isReachable(RobotSide robotSide, FramePoint3DReadOnly contactPoint)
   {
      return isReachable(robotSide, contactPoint, 0.0);
   }

   public boolean isReachable(RobotSide robotSide, FramePoint3DReadOnly contactPoint, double epsilon)
   {
      return isReachable(robotSide, contactPoint, centroidalFrame, tempPoint, epsilon);
   }

   public static boolean isReachable(RobotSide robotSide, FramePoint3DReadOnly contactPoint, ReferenceFrame centroidalFrame, FramePoint3D shoulderToContactPoint, double epsilon)
   {
      shoulderToContactPoint.setToZero(centroidalFrame);
      shoulderToContactPoint.setMatchingFrame(contactPoint);
      shoulderToContactPoint.sub(SHOULDER_COM_OFFSET_X, robotSide.negateIfRightSide(SHOULDER_COM_OFFSET_Y), SHOULDER_COM_OFFSET_Z);

      double shoulderToContactPointNorm = shoulderToContactPoint.norm();
      if (shoulderToContactPointNorm < REACHABILITY_RADIUS_MIN - epsilon)
         return false;
      if (shoulderToContactPointNorm > epsilon + REACHABILITY_RADIUS_MAX)
         return false;
      if (robotSide.negateIfRightSide(shoulderToContactPoint.getY()) < -MAX_INWARD_DISTANCE - epsilon)
         return false;

      return true;
   }

   public static double getArmExtension(RobotSide robotSide, FramePoint3DReadOnly contactPoint, ReferenceFrame centroidalFrame, FramePoint3D shoulderToContactPoint)
   {
      shoulderToContactPoint.setToZero(centroidalFrame);
      shoulderToContactPoint.setMatchingFrame(contactPoint);
      shoulderToContactPoint.sub(SHOULDER_COM_OFFSET_X, robotSide.negateIfRightSide(SHOULDER_COM_OFFSET_Y), SHOULDER_COM_OFFSET_Z);

      double shoulderToContactPointNorm = shoulderToContactPoint.norm();
      return shoulderToContactPointNorm;
   }

   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("comPosition", centroidalPose.getPosition(), 0.02, ColorDefinitions.Blue()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("comVelocity", centroidalPose.getPosition(), comVelocity, 0.4, ColorDefinitions.Blue()));

      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("comPosition2d", centroidalPose.getPosition(), 0.01, ColorDefinitions.Blue(), DefaultPoint2DGraphic.CIRCLE_CROSS));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("capturePointPosition2d", capturePointPosition, 0.01, ColorDefinitions.Red(), DefaultPoint2DGraphic.CIRCLE));

      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("nominalFootPolygon", nominalFootPolygon, ColorDefinitions.Black(), false));

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "_";
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(sidePrefix + "HandPosition", handPositions.get(robotSide), 0.02, ColorDefinitions.Red()));

//         ColorDefinition color = ColorDefinitions.argb(ContinuousStepGenerator.defaultFeetColors.get(robotSide).getRGB());
//         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygonExtruded3DDefinition(sidePrefix + "Foothold", solePoses.get(robotSide), defaultFootPolygons.get(robotSide), 0.01, color));
      }

      return group;
   }
}
