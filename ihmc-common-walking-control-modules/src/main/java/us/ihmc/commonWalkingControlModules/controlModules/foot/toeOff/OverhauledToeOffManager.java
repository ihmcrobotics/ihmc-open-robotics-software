package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;

public class OverhauledToeOffManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private Footstep nextFootstep;

   private final SideDependentList<ConvexPolygon2DReadOnly> footDefaultPolygons;
   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;
   private final SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorldFrame;

   private final FrameConvexPolygon2D leadingFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D trailingFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D onToesSupportPolygon = new FrameConvexPolygon2D();

   private final ToeOffStepPositionInspector stepPositionInspector;
   private final DynamicStateInspector dynamicStateInspector;
   private final LegJointLimitsInspector jointLimitsInspector;

   private final ToeOffCalculator toeOffCalculator;

   private final DynamicStateInspectorParameters dynamicStateInspectorParameters;
   private final BooleanProvider doToeOffIfPossibleInDoubleSupport;
   private final BooleanProvider doToeOffIfPossibleInSingleSupport;

   private final YoBoolean doToeOff = new YoBoolean("doToeOff", registry);

   private final FramePose3D nextFrontFootPose = new FramePose3D();
   private final FrameLineSegment2D toeOffLine = new FrameLineSegment2D();
   private final FramePoint2D toeOffPoint = new FramePoint2D();
   private final FramePoint2D tmpPoint2d = new FramePoint2D();

   public OverhauledToeOffManager(WalkingControllerParameters walkingControllerParameters,
                                  DynamicStateInspectorParameters dynamicStateInspectorParameters,
                                  SideDependentList<MovingReferenceFrame> soleZUpFrames,
                                  SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorldFrame,
                                  FullHumanoidRobotModel fullRobotModel,
                                  ToeOffCalculator toeOffCalculator,
                                  YoRegistry parentRegistry)
   {
      this(walkingControllerParameters.getToeOffParameters(),
           walkingControllerParameters.getSteppingParameters(),
           dynamicStateInspectorParameters,
           soleZUpFrames,
           footPolygonsInWorldFrame,
           fullRobotModel,
           toeOffCalculator,
           parentRegistry);
   }

   public OverhauledToeOffManager(ToeOffParameters toeOffParameters,
                                  SteppingParameters steppingParameters,
                                  DynamicStateInspectorParameters dynamicStateInspectorParameters,
                                  SideDependentList<MovingReferenceFrame> soleZUpFrames,
                                  SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorldFrame,
                                  FullHumanoidRobotModel fullRobotModel,
                                  ToeOffCalculator toeOffCalculator,
                                  YoRegistry parentRegistry)
   {
      this.toeOffCalculator = toeOffCalculator;
      this.soleZUpFrames = soleZUpFrames;
      this.footPolygonsInWorldFrame = footPolygonsInWorldFrame;
      this.dynamicStateInspectorParameters = dynamicStateInspectorParameters;

      double footLength = steppingParameters.getFootBackwardOffset() + steppingParameters.getFootForwardOffset();

      stepPositionInspector = new ToeOffStepPositionInspector(soleZUpFrames, toeOffParameters, steppingParameters.getInPlaceWidth(), footLength, registry);
      dynamicStateInspector = new DynamicStateInspector(registry);
      if (fullRobotModel != null)
         jointLimitsInspector = new LegJointLimitsInspector(toeOffParameters, fullRobotModel, registry);
      else
         jointLimitsInspector = null;

      footDefaultPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2D polygon = new FrameConvexPolygon2D(footPolygonsInWorldFrame.get(robotSide));
         polygon.changeFrameAndProjectToXYPlane(soleZUpFrames.get(robotSide));
         footDefaultPolygons.put(robotSide, new ConvexPolygon2D(polygon));
      }

      doToeOffIfPossibleInDoubleSupport = new BooleanParameter("doToeOffIfPossibleInDoubleSupport", registry, toeOffParameters.doToeOffIfPossible());
      doToeOffIfPossibleInSingleSupport = new BooleanParameter("doToeOffIfPossibleInSingleSupport",
                                                               registry,
                                                               toeOffParameters.doToeOffIfPossibleInSingleSupport());

      parentRegistry.addChild(registry);
   }

   public void submitNextFootstep(Footstep nextFootstep, Footstep nextNextFootstep)
   {
      this.nextFootstep = nextFootstep;
   }

   public void updateToeOffStatusSingleSupport(FramePoint3DReadOnly exitCMP,
                                               FramePoint2DReadOnly desiredECMP,
                                               FramePoint2DReadOnly desiredCoP,
                                               FramePoint2DReadOnly desiredICP,
                                               FramePoint2DReadOnly currentICP,
                                               FramePoint2DReadOnly finalDesiredICP)
   {
      setLeadingPolygonFromNextFootstep(leadingFootSupportPolygon);

      RobotSide trailingLeg = nextFootstep.getRobotSide().getOppositeSide();
      setLeadingPolygonFromSupportFoot(trailingLeg, trailingFootSupportPolygon);

      FramePoint2DReadOnly toeOffPoint = computeOnToesSupportPolygon(exitCMP, desiredECMP, trailingLeg, leadingFootSupportPolygon);

      //      if (finalDesiredICP != null && !onToesSupportPolygon.isPointInside(finalDesiredICP))
      //      { // This allows to better account for long and/or fast steps when the final ICP lies outside the toe-off support polygon.
      //         onToesSupportPolygon.addVertex(finalDesiredICP);
      //         onToesSupportPolygon.update();
      //      }

      updateToeOffConditions(trailingLeg,
                             desiredICP,
                             currentICP,
                             toeOffPoint,
                             doToeOffIfPossibleInSingleSupport.getValue(),
                             nextFootstep.getFootstepPose());
   }

   public void updateToeOffStatusDoubleSupport(RobotSide trailingLeg,
                                               FramePoint3DReadOnly exitCMP,
                                               FramePoint2DReadOnly desiredECMP,
                                               FramePoint2DReadOnly desiredCoP,
                                               FramePoint2DReadOnly desiredICP,
                                               FramePoint2DReadOnly currentICP,
                                               FramePoint2DReadOnly finalDesiredICP)
   {
      setLeadingPolygonFromSupportFoot(trailingLeg, trailingFootSupportPolygon);
      setLeadingPolygonFromSupportFoot(trailingLeg.getOppositeSide(), leadingFootSupportPolygon);

      FramePoint2DReadOnly toeOffPoint = computeOnToesSupportPolygon(exitCMP, desiredECMP, trailingLeg, leadingFootSupportPolygon);

      //      if (finalDesiredICP != null && !onToesSupportPolygon.isPointInside(finalDesiredICP))
      //      { // This allows to better account for long and/or fast steps when the final ICP lies outside the toe-off support polygon.
      //         onToesSupportPolygon.addVertex(finalDesiredICP);
      //         onToesSupportPolygon.update();
      //      }

      nextFrontFootPose.setToZero(soleZUpFrames.get(trailingLeg.getOppositeSide()));
      nextFrontFootPose.changeFrame(worldFrame);

      updateToeOffConditions(trailingLeg,
                             desiredICP,
                             currentICP,
                             toeOffPoint,
                             doToeOffIfPossibleInDoubleSupport.getValue(),
                             nextFrontFootPose);
   }

   private void updateToeOffConditions(RobotSide trailingLeg,
                                       FramePoint2DReadOnly desiredICP,
                                       FramePoint2DReadOnly currentICP,
                                       FramePoint2DReadOnly toeOffPoint,
                                       boolean allowToeOff,
                                       FramePose3DReadOnly nextFrontFootPose)
   {
      dynamicStateInspector.setPolygons(leadingFootSupportPolygon, trailingFootSupportPolygon, onToesSupportPolygon);
      boolean wellPositioned = stepPositionInspector.isFrontFootWellPositionedForToeOff(trailingLeg, nextFrontFootPose);
      dynamicStateInspector.checkICPLocations(dynamicStateInspectorParameters, trailingLeg, nextFrontFootPose, desiredICP, currentICP, toeOffPoint);

      boolean jointsNeedToeingOff = false;
      if (jointLimitsInspector != null)
      {
         jointLimitsInspector.updateToeOffConditions(trailingLeg);
         jointsNeedToeingOff = jointLimitsInspector.needToSwitchToToeOffDueToJointLimit();
      }
      boolean doToeOff = allowToeOff && wellPositioned;
      doToeOff &= (jointsNeedToeingOff || dynamicStateInspector.areDynamicsOkForToeOff());

      this.doToeOff.set(doToeOff);
   }

   private void setLeadingPolygonFromSupportFoot(RobotSide side, FrameConvexPolygon2DBasics polygonToPack)
   {
      if (footPolygonsInWorldFrame != null && footPolygonsInWorldFrame.get(side).getNumberOfVertices() > 0)
      {
         polygonToPack.setIncludingFrame(footPolygonsInWorldFrame.get(side));
      }
      else
      {
         polygonToPack.setIncludingFrame(soleZUpFrames.get(side), footDefaultPolygons.get(side));
         polygonToPack.changeFrameAndProjectToXYPlane(worldFrame);
      }
   }

   private void setLeadingPolygonFromNextFootstep(FrameConvexPolygon2DBasics polygonToPack)
   {
      if (nextFootstep == null || nextFootstep.getRobotSide() == null)
         throw new RuntimeException("Next footstep has not been set");

      ReferenceFrame footstepSoleFrame = nextFootstep.getSoleReferenceFrame();
      List<Point2D> predictedContactPoints = nextFootstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         polygonToPack.clear(footstepSoleFrame);
         for (int i = 0; i < predictedContactPoints.size(); i++)
            polygonToPack.addVertex(predictedContactPoints.get(i));
         polygonToPack.update();
      }
      else
      {
         ConvexPolygon2DReadOnly footPolygon = footDefaultPolygons.get(nextFootstep.getRobotSide());
         polygonToPack.setIncludingFrame(footstepSoleFrame, footPolygon);
      }
      polygonToPack.changeFrameAndProjectToXYPlane(worldFrame);
   }

   public FramePoint2DReadOnly computeOnToesSupportPolygon(FramePoint3DReadOnly exitCMP,
                                                           FramePoint2DReadOnly desiredECMP,
                                                           RobotSide trailingSide,
                                                           FrameConvexPolygon2DReadOnly leadingSupportPolygon)
   {
      if (exitCMP == null)
         computeToeContactFromFrontEdge(trailingSide);
      else
         computeToeContactsUsingCalculator(exitCMP, desiredECMP, trailingSide);

      onToesSupportPolygon.clear(worldFrame);
      onToesSupportPolygon.addVerticesMatchingFrame(leadingSupportPolygon, false);
      onToesSupportPolygon.addVertexMatchingFrame(toeOffPoint, false);
      onToesSupportPolygon.update();

      return toeOffPoint;
   }

   private void computeToeContactsUsingCalculator(FramePoint3DReadOnly exitCMP, FramePoint2DReadOnly desiredECMP, RobotSide supportSide)
   {
      toeOffCalculator.setExitCMP(exitCMP, supportSide);
      toeOffCalculator.computeToeOffContactPoint(desiredECMP, supportSide);

      toeOffPoint.setToZero(soleZUpFrames.get(supportSide));
      toeOffCalculator.getToeOffContactPoint(toeOffPoint, supportSide);
   }

   protected void computeToeContactFromFrontEdge(RobotSide supportSide)
   {
      ConvexPolygon2DReadOnly footDefaultPolygon = footDefaultPolygons.get(supportSide);
      ReferenceFrame referenceFrame = soleZUpFrames.get(supportSide);
      toeOffLine.getFirstEndpoint().set(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);
      toeOffLine.getSecondEndpoint().set(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);

      // gets the leading two toe points
      for (int i = 0; i < footDefaultPolygon.getNumberOfVertices(); i++)
      {
         tmpPoint2d.setIncludingFrame(referenceFrame, footDefaultPolygon.getVertex(i));
         if (tmpPoint2d.getX() > toeOffLine.getFirstEndpoint().getX())
         { // further ahead than leading point
            toeOffLine.getSecondEndpoint().set(toeOffLine.getFirstEndpoint());
            toeOffLine.getFirstEndpoint().set(tmpPoint2d);
         }
         else if (tmpPoint2d.getX() > toeOffLine.getSecondEndpoint().getX())
         { // further ahead than second leading point
            toeOffLine.getSecondEndpoint().set(tmpPoint2d);
         }
      }

      toeOffPoint.setToZero(referenceFrame);
      toeOffLine.midpoint(toeOffPoint);
   }
}
