package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.List;

public class GeometricToeOffManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   // external states to use
   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;
   private final SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorldFrame;

   // calculators
   private final ToeOffStepPositionInspector stepPositionInspector;
   private final DynamicStateInspector dynamicStateInspector;
   private final LegJointLimitsInspector jointLimitsInspector;

   private final ToeOffCalculator toeOffCalculator;

   // parameters
   private final SideDependentList<ConvexPolygon2DReadOnly> footDefaultPolygons;
   private final SideDependentList<FramePoint2DReadOnly> defaultToeOffPoints = new SideDependentList<>();

   private final DynamicStateInspectorParameters dynamicStateInspectorParameters;
   private final BooleanProvider doToeOffIfPossibleInDoubleSupport;
   private final BooleanProvider doToeOffIfPossibleInSingleSupport;

   // state calculation
   private final YoBoolean doToeOff = new YoBoolean("doToeOff", registry);

   // polygons used for all the calculations
   private final FrameConvexPolygon2D leadingFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D trailingFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D onToesSupportPolygon = new FrameConvexPolygon2D();
   private final FramePoint2D toeOffPoint = new FramePoint2D();

   // temp objects
   private final PoseReferenceFrame nextStepFrame = new PoseReferenceFrame("nextStepFrame", worldFrame);
   private final FramePose3D nextFrontFootPose = new FramePose3D();

   public GeometricToeOffManager(HighLevelHumanoidControllerToolbox controllerToolbox,
                                 WalkingControllerParameters walkingControllerParameters,
                                 DynamicStateInspectorParameters dynamicStateInspectorParameters,
                                 ToeOffCalculator toeOffCalculator,
                                 YoRegistry parentRegistry)
   {
      this(walkingControllerParameters,
           dynamicStateInspectorParameters,
           controllerToolbox.getReferenceFrames().getSoleZUpFrames(),
           controllerToolbox.getBipedSupportPolygons().getFootPolygonsInWorldFrame(),
           controllerToolbox.getDefaultFootPolygons(),
           controllerToolbox.getFullRobotModel(),
           toeOffCalculator,
           parentRegistry);
   }

   public GeometricToeOffManager(WalkingControllerParameters walkingControllerParameters,
                                 DynamicStateInspectorParameters dynamicStateInspectorParameters,
                                 SideDependentList<MovingReferenceFrame> soleZUpFrames,
                                 SideDependentList<? extends FrameConvexPolygon2DReadOnly> footPolygonsInWorldFrame,
                                 SideDependentList<? extends FrameConvexPolygon2DReadOnly> footDefaultPolygons,
                                 FullHumanoidRobotModel fullRobotModel,
                                 ToeOffCalculator toeOffCalculator,
                                 YoRegistry parentRegistry)
   {
      this.toeOffCalculator = toeOffCalculator;
      this.soleZUpFrames = soleZUpFrames;
      this.footPolygonsInWorldFrame = footPolygonsInWorldFrame;
      this.dynamicStateInspectorParameters = dynamicStateInspectorParameters;

      ToeOffParameters toeOffParameters = walkingControllerParameters.getToeOffParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();

      double footLength = steppingParameters.getFootBackwardOffset() + steppingParameters.getFootForwardOffset();

      stepPositionInspector = new ToeOffStepPositionInspector(soleZUpFrames,
                                                              walkingControllerParameters,
                                                              toeOffParameters,
                                                              steppingParameters.getInPlaceWidth(),
                                                              footLength,
                                                              registry);
      dynamicStateInspector = new DynamicStateInspector(registry);
      if (fullRobotModel != null)
         jointLimitsInspector = new LegJointLimitsInspector(toeOffParameters, fullRobotModel, registry);
      else
         jointLimitsInspector = null;

      this.footDefaultPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2D polygon = new FrameConvexPolygon2D(footDefaultPolygons.get(robotSide));
         if (polygon.isEmpty())
            throw new RuntimeException("No default polygon was specified.");
         polygon.changeFrameAndProjectToXYPlane(soleZUpFrames.get(robotSide));

         ConvexPolygon2DReadOnly defaultPolygon = new ConvexPolygon2D(polygon);
         this.footDefaultPolygons.put(robotSide, defaultPolygon);
         defaultToeOffPoints.put(robotSide, new FramePoint2D(soleZUpFrames.get(robotSide), computeMiddleOfFrontEdge(defaultPolygon)));
      }

      doToeOffIfPossibleInDoubleSupport = new BooleanParameter("doToeOffIfPossibleInDoubleSupport", registry, toeOffParameters.doToeOffIfPossible());
      doToeOffIfPossibleInSingleSupport = new BooleanParameter("doToeOffIfPossibleInSingleSupport",
                                                               registry,
                                                               toeOffParameters.doToeOffIfPossibleInSingleSupport());

      parentRegistry.addChild(registry);
   }

   private static Point2DReadOnly computeMiddleOfFrontEdge(ConvexPolygon2DReadOnly polygon)
   {
      LineSegment2D frontEdge = new LineSegment2D();
      frontEdge.getFirstEndpoint().set(Double.NEGATIVE_INFINITY, 0.0);
      frontEdge.getSecondEndpoint().set(Double.NEGATIVE_INFINITY, 0.0);

      // gets the leading two toe points
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = polygon.getVertex(i);
         if (polygon.getVertex(i).getX() > frontEdge.getFirstEndpoint().getX())
         { // further ahead than leading point
            frontEdge.getSecondEndpoint().set(frontEdge.getFirstEndpoint());
            frontEdge.getFirstEndpoint().set(vertex);
         }
         else if (vertex.getX() > frontEdge.getSecondEndpoint().getX())
         { // further ahead than second leading point
            frontEdge.getSecondEndpoint().set(vertex);
         }
      }

      return frontEdge.midpoint();
   }

   public boolean isSteppingUp()
   {
      return stepPositionInspector.isSteppingUp();
   }

   public void reset()
   {
      doToeOff.set(false);

      jointLimitsInspector.reset();
      stepPositionInspector.reset();
      dynamicStateInspector.reset();
   }


   public void updateToeOffStatusSingleSupport(RobotSide stepSide,
                                               FramePose3DReadOnly footstepPose,
                                               List<Point2D> predictedContactPoints,
                                               FramePoint3DReadOnly exitCMP,
                                               FramePoint2DReadOnly desiredECMP,
                                               FramePoint2DReadOnly desiredICP,
                                               FramePoint2DReadOnly currentICP)
   {
      setLeadingPolygonFromNextFootstep(stepSide, footstepPose, predictedContactPoints, leadingFootSupportPolygon);

      RobotSide trailingLeg = stepSide.getOppositeSide();
      setLeadingPolygonFromSupportFoot(trailingLeg, trailingFootSupportPolygon);

      FramePoint2DReadOnly toeOffPoint = computeOnToesSupportPolygon(exitCMP, desiredECMP, trailingLeg, leadingFootSupportPolygon);

      updateToeOffConditions(trailingLeg,
                             desiredICP,
                             currentICP,
                             toeOffPoint,
                             doToeOffIfPossibleInSingleSupport.getValue(),
                             footstepPose);
   }

   public void updateToeOffStatusDoubleSupport(RobotSide trailingLeg,
                                               FramePoint3DReadOnly exitCMP,
                                               FramePoint2DReadOnly desiredECMP,
                                               FramePoint2DReadOnly desiredICP,
                                               FramePoint2DReadOnly currentICP)
   {
      setLeadingPolygonFromSupportFoot(trailingLeg, trailingFootSupportPolygon);
      setLeadingPolygonFromSupportFoot(trailingLeg.getOppositeSide(), leadingFootSupportPolygon);

      FramePoint2DReadOnly toeOffPoint = computeOnToesSupportPolygon(exitCMP, desiredECMP, trailingLeg, leadingFootSupportPolygon);

      nextFrontFootPose.setToZero(soleZUpFrames.get(trailingLeg.getOppositeSide()));
      nextFrontFootPose.changeFrame(worldFrame);

      updateToeOffConditions(trailingLeg,
                             desiredICP,
                             currentICP,
                             toeOffPoint,
                             doToeOffIfPossibleInDoubleSupport.getValue(),
                             nextFrontFootPose);
   }

   public boolean areFeetWellPositionedForToeOff(RobotSide trailingLegSide)
   {
      nextFrontFootPose.setToZero(soleZUpFrames.get(trailingLegSide.getOppositeSide()));
      nextFrontFootPose.changeFrame(worldFrame);
      return areFeetWellPositionedForToeOff(trailingLegSide, nextFrontFootPose);
   }

   public boolean areFeetWellPositionedForToeOff(RobotSide trailingLegSide, FramePose3DReadOnly nextFrontFootPose)
   {
      return stepPositionInspector.isFrontFootWellPositionedForToeOff(trailingLegSide, nextFrontFootPose);
   }

   public boolean doToeOff()
   {
      return doToeOff.getBooleanValue();
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
      boolean doToeOff = allowToeOff && wellPositioned && !dynamicStateInspector.areDynamicsDefinitelyNotOkForToeOff();
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
      }
      polygonToPack.changeFrameAndProjectToXYPlane(worldFrame);
   }

   private void setLeadingPolygonFromNextFootstep(RobotSide stepSide,
                                                  FramePose3DReadOnly footstepPose,
                                                  List<Point2D> predictedContactPoints,
                                                  FrameConvexPolygon2DBasics polygonToPack)
   {
      nextStepFrame.setPoseAndUpdate(footstepPose);

      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         polygonToPack.clear(nextStepFrame);
         for (int i = 0; i < predictedContactPoints.size(); i++)
            polygonToPack.addVertex(predictedContactPoints.get(i));
         polygonToPack.update();
      }
      else
      {
         ConvexPolygon2DReadOnly footPolygon = footDefaultPolygons.get(stepSide);
         polygonToPack.setIncludingFrame(nextStepFrame, footPolygon);
      }
      polygonToPack.changeFrameAndProjectToXYPlane(worldFrame);
   }

   private FramePoint2DReadOnly computeOnToesSupportPolygon(FramePoint3DReadOnly exitCMP,
                                                           FramePoint2DReadOnly desiredECMP,
                                                           RobotSide trailingSide,
                                                           FrameConvexPolygon2DReadOnly leadingSupportPolygon)
   {
      if (exitCMP == null)
         toeOffPoint.setIncludingFrame(defaultToeOffPoints.get(trailingSide));
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
}
