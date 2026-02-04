package us.ihmc.avatar.multiContact.pushRecovery;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.PoseReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
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
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;

/**
 * Reduced order robot model consisting of the centroidal and contact states.
 */
public class ReducedOrderRobotModel
{
   public static final double MAX_HAND_ACCELERATION = 15.0;

   /* Nominal offset in mid-feet zup frame from CoM to shoulder position, taken at default standing home pose */
   private static final double SHOULDER_COM_OFFSET_X = -0.02;
   private static final double SHOULDER_COM_OFFSET_Y = 0.12;
   private static final double SHOULDER_COM_OFFSET_Z = 0.44;

   /* Maximum allowed reachablity distance from shoulder to hand */
   public static final double REACHABILITY_RADIUS_MAX = 0.83;

   /* Maximum inward reaching distance, to prevent too much cross-over */
   public static final double MAX_INWARD_DISTANCE = 0.2;

   public static final double NOMINAL_COM_HEIGHT = 0.9;
   public static final double OMEGA = Math.sqrt(9.81 / NOMINAL_COM_HEIGHT);

   private final SegmentDependentList<RobotSide, ArrayList<Point2D>> footContactPoints;
   private final YoFrameConvexPolygon2D nominalFootPolygon;

   private final YoFramePoint3D comPosition;
   private final YoFrameVector3D comVelocity;

   private final YoFramePoint2D capturePointPosition;

   private final SideDependentList<YoFramePose3D> solePoses = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> soleFrames = new SideDependentList<>();
   private final MidFrameZUpFrame midFeetZUpFrame;

   private final SideDependentList<YoBoolean> handInContact = new SideDependentList<>();
   private final SideDependentList<YoFramePoint3D> handPositions = new SideDependentList<>();
   private final SideDependentList<FramePoint3D> shoulderPositions = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final SideDependentList<YoFrameVector3D> handVelocities = new SideDependentList<>();
   private final SideDependentList<YoFrameVector3D> handContactNormals = new SideDependentList<>();

   public ReducedOrderRobotModel(RobotContactPointParameters<RobotSide> contactPointParameters, YoRegistry registry)
   {
      footContactPoints = contactPointParameters.getFootContactPoints();
      comPosition = new YoFramePoint3D("comPosition", ReferenceFrame.getWorldFrame(), registry);
      comVelocity = new YoFrameVector3D("comVelocity", ReferenceFrame.getWorldFrame(), registry);
      capturePointPosition = new YoFramePoint2D("capturePointPosition", ReferenceFrame.getWorldFrame(), registry);
      nominalFootPolygon = new YoFrameConvexPolygon2D("nominalFootPolygon", ReferenceFrame.getWorldFrame(), 8, registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "_";

         solePoses.put(robotSide, new YoFramePose3D(sidePrefix + "SolePose", ReferenceFrame.getWorldFrame(), registry));
         soleFrames.put(robotSide, new PoseReferenceFrame(sidePrefix + "SoleFrame", ReferenceFrame.getWorldFrame()));

         handInContact.put(robotSide, new YoBoolean(sidePrefix + "HandInContact", registry));
         handPositions.put(robotSide, new YoFramePoint3D(sidePrefix + "HandPosition", ReferenceFrame.getWorldFrame(), registry));
         handVelocities.put(robotSide, new YoFrameVector3D(sidePrefix + "HandVelocity", ReferenceFrame.getWorldFrame(), registry));
         handContactNormals.put(robotSide, new YoFrameVector3D(sidePrefix + "HandContactNormal", ReferenceFrame.getWorldFrame(), registry));
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZUpFrame", ReferenceFrame.getWorldFrame(), soleFrames.get(RobotSide.LEFT), soleFrames.get(RobotSide.RIGHT));
   }

   public void set(ReducedOrderRobotModel other)
   {
      this.nominalFootPolygon.set(other.nominalFootPolygon);
      this.comPosition.set(other.comPosition);
      this.comVelocity.set(other.comVelocity);

      for (RobotSide robotSide : RobotSide.values)
      {
         solePoses.get(robotSide).set(other.solePoses.get(robotSide));
         handPositions.get(robotSide).set(other.handPositions.get(robotSide));
         handVelocities.get(robotSide).set(other.handVelocities.get(robotSide));
         handInContact.get(robotSide).set(other.handInContact.get(robotSide).getValue());
         handContactNormals.get(robotSide).set(other.handContactNormals.get(robotSide));
      }

      updateFootFrames();
      updateShoulders();
      updateSupportPolygon();
      updateCapturePoint();
   }

   private void updateFootFrames()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.get(robotSide).setPoseAndUpdate(solePoses.get(robotSide));
      }

      midFeetZUpFrame.update();
   }

   private void updateShoulders()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint3D shoulder = shoulderPositions.get(robotSide);
         shoulder.setIncludingFrame(comPosition);
         shoulder.changeFrame(midFeetZUpFrame);
         shoulder.add(SHOULDER_COM_OFFSET_X, robotSide.negateIfRightSide(SHOULDER_COM_OFFSET_Y), SHOULDER_COM_OFFSET_Z);
      }
   }

   private void updateSupportPolygon()
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
      capturePointPosition.add(comPosition.getX(), comPosition.getY());
   }

   public boolean isHandInContact(RobotSide robotSide)
   {
      return handInContact.get(robotSide).getValue();
   }

   public FrameVector3DReadOnly getHandContactNormal(RobotSide robotSide)
   {
      return handContactNormals.get(robotSide);
   }

   public void setHandInContact(RobotSide robotSide, FrameVector3DReadOnly contactNormal)
   {
      handInContact.get(robotSide).set(true);
      handContactNormals.get(robotSide).set(contactNormal);
   }

   public FixedFramePoint3DBasics getHandPosition(RobotSide robotSide)
   {
      return handPositions.get(robotSide);
   }

   public FixedFrameVector3DBasics getHandVelocity(RobotSide robotSide)
   {
      return handVelocities.get(robotSide);
   }

   public FramePoint3DReadOnly getShoulderPosition(RobotSide robotSide)
   {
      return shoulderPositions.get(robotSide);
   }

   public FixedFramePoint3DBasics getComPosition()
   {
      return comPosition;
   }

   public FixedFrameVector3DBasics getComVelocity()
   {
      return comVelocity;
   }

   public ReferenceFrame getMidFeetZUpFrame()
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
      this.comPosition.setFromReferenceFrame(referenceFrames.getCenterOfMassFrame());
      this.comVelocity.setMatchingFrame(comVelocity);

      for (RobotSide robotSide : RobotSide.values)
      {
         this.solePoses.get(robotSide).setFromReferenceFrame(referenceFrames.getSoleFrame(robotSide));
         this.handPositions.get(robotSide).setFromReferenceFrame(fullRobotModel.getHandControlFrame(robotSide));
         this.handVelocities.get(robotSide).setToZero();
         this.handInContact.get(robotSide).set(false);
         this.handContactNormals.get(robotSide).setToNaN();
      }

      updateFootFrames();
      updateShoulders();
      updateSupportPolygon();
      updateCapturePoint();
   }

   public boolean isReachable(RobotSide robotSide, FramePoint3DReadOnly contactPoint)
   {
      FramePoint3D shoulderToContactPoint = new FramePoint3D(midFeetZUpFrame);
      shoulderToContactPoint.setMatchingFrame(contactPoint);
      shoulderToContactPoint.sub(shoulderPositions.get(robotSide));

      double shoulderToContactPointNorm = shoulderToContactPoint.norm();
      if (shoulderToContactPointNorm > REACHABILITY_RADIUS_MAX)
         return false;
      if (robotSide.negateIfRightSide(shoulderToContactPoint.getY()) < -MAX_INWARD_DISTANCE)
         return false;

      return true;
   }

   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("comPosition", comPosition, 0.02, ColorDefinitions.Blue()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("comVelocity", comPosition, comVelocity, 0.4, ColorDefinitions.Blue()));

      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("comPosition2d", comPosition, 0.01, ColorDefinitions.Blue(), DefaultPoint2DGraphic.CIRCLE_CROSS));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D("capturePointPosition2d", capturePointPosition, 0.01, ColorDefinitions.Red(), DefaultPoint2DGraphic.CIRCLE));

      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("nominalFootPolygon", nominalFootPolygon, ColorDefinitions.Black(), false));

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "_";
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D(sidePrefix + "SolePose", solePoses.get(robotSide), 0.3, ColorDefinitions.Red()));
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(sidePrefix + "HandPosition", handPositions.get(robotSide), 0.03, ColorDefinitions.Red()));
      }

      return group;
   }
}
