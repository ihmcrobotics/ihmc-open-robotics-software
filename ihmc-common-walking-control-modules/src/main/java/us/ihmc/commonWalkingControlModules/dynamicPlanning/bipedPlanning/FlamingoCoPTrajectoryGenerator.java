package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;
import java.util.function.Supplier;

public class FlamingoCoPTrajectoryGenerator extends CoPTrajectoryGenerator
{
   private final CoPTrajectoryParameters parameters;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> movingPolygonsInSole = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final FrameConvexPolygon2DBasics nextPolygon = new FrameConvexPolygon2D();
   private final PoseReferenceFrame nextStepFrame;
   private final SideDependentList<PoseReferenceFrame> stepFrames = new SideDependentList<>();

   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final FramePose3D tempPose = new FramePose3D();

   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final FramePoint2D tempPointForCoPCalculation = new FramePoint2D();
   private final FramePoint2D midfootCoP = new FramePoint2D();

   private WaypointViewer viewer = null;

   public FlamingoCoPTrajectoryGenerator(CoPTrajectoryParameters parameters, ConvexPolygon2DReadOnly defaultSupportPolygon, YoRegistry parentRegistry)
   {
      super(FlamingoCoPTrajectoryGenerator.class, parentRegistry);

      this.parameters = parameters;
      this.defaultSupportPolygon.set(defaultSupportPolygon);

      registry = new YoRegistry(getClass().getSimpleName());

      for (RobotSide robotSide : RobotSide.values)
      {
         stepFrames.put(robotSide, new PoseReferenceFrame(robotSide.getLowerCaseName() + "StepFrame", ReferenceFrame.getWorldFrame()));
      }
      nextStepFrame = new PoseReferenceFrame("nextStepFrame", ReferenceFrame.getWorldFrame());

      parentRegistry.addChild(registry);
      clear();
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public void setWaypointViewer(WaypointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void clear()
   {
      contactStateProviders.clear();
   }

   private void reset()
   {
      contactStateProviders.clear();
   }

   public void set(Point2DReadOnly constantCop)
   {
      clear();

      SettableContactStateProvider contactState = contactStateProviders.add();
      contactState.getTimeInterval().setInterval(0.0, Double.POSITIVE_INFINITY);
      contactState.setStartCopPosition(constantCop);
      contactState.setEndCopPosition(constantCop);
   }


   public void compute(CoPTrajectoryGeneratorState state)
   {
      contactStateProviders.clear();

      // Add initial support states of the feet and set the moving polygons
      for (RobotSide robotSide : RobotSide.values)
      {
         // Record the initial step frames. In case there is a step touching down this frame will be updated.
         PoseReferenceFrame stepFrame = stepFrames.get(robotSide);
         tempPose.setIncludingFrame(state.getFootPose(robotSide));
         tempPose.changeFrame(stepFrame.getParent());
         stepFrame.setPoseAndUpdate(tempPose);

         movingPolygonsInSole.get(robotSide).setIncludingFrame(state.getFootPolygonInSole(robotSide));
         movingPolygonsInSole.get(robotSide).changeFrameAndProjectToXYPlane(stepFrame);
      }

      // compute cop waypoint location
      SettableContactStateProvider contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartTime(0.0);
      contactStateProvider.setStartCopPosition(state.getInitialCoP());

      PlanningFootstep footstep = state.getFootstep(0);
      PlanningTiming timings = state.getTiming(0);
      PlanningShiftFraction shiftFraction = state.getShiftFraction(0);
      RobotSide swingSide = footstep.getRobotSide();
      RobotSide supportSide = swingSide.getOppositeSide();

      FrameConvexPolygon2DReadOnly previousPolygon = movingPolygonsInSole.get(swingSide);
      FrameConvexPolygon2DReadOnly currentPolygon = movingPolygonsInSole.get(swingSide.getOppositeSide());

      nextStepFrame.setPoseAndUpdate(footstep.getFootstepPose());
      extractSupportPolygon(footstep, nextStepFrame, nextPolygon, defaultSupportPolygon);

      computeEntryCoPPointLocation(tempPointForCoPCalculation, previousPolygon, nextPolygon, supportSide);
      midfootCoP.interpolate(state.getInitialCoP(), tempPointForCoPCalculation, shiftFraction.getTransferWeightDistribution());

      contactStateProvider.setDuration(shiftFraction.getTransferSplitFraction() * timings.getTransferTime());
      contactStateProvider.setEndCopPosition(midfootCoP);

      SettableContactStateProvider previousContactState = contactStateProvider;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);
      contactStateProvider.setDuration((1.0 - shiftFraction.getTransferSplitFraction()) * timings.getTransferTime());
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);

      previousContactState = contactStateProvider;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);
      contactStateProvider.setDuration(Double.POSITIVE_INFINITY);
      tempPointForCoPCalculation.setIncludingFrame(currentPolygon.getCentroid());
      tempPointForCoPCalculation.changeFrameAndProjectToXYPlane(worldFrame);
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);

      if (viewer != null)
         viewer.updateWaypoints(contactStateProviders);
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }

   private void extractSupportPolygon(PlanningFootstep footstep,
                                      ReferenceFrame stepFrame,
                                      FrameConvexPolygon2DBasics footSupportPolygonToPack,
                                      ConvexPolygon2DReadOnly defaultSupportPolygon)
   {
      if (footstep.hasPredictedContactPoints())
      {
         List<? extends Point2DReadOnly> predictedContactPoints = footstep.getPredictedContactPoints();

         footSupportPolygonToPack.clear(stepFrame);
         for (int i = 0; i < predictedContactPoints.size(); i++)
            footSupportPolygonToPack.addVertex(predictedContactPoints.get(i));
         footSupportPolygonToPack.update();
      }
      else
      {
         footSupportPolygonToPack.setIncludingFrame(stepFrame, defaultSupportPolygon);
      }
   }

   private void computeEntryCoPPointLocation(FramePoint2DBasics copLocationToPack,
                                             FrameConvexPolygon2DReadOnly previousFootPolygon,
                                             FrameConvexPolygon2DReadOnly footPolygon,
                                             RobotSide supportSide)
   {
      computeCoPLocation(copLocationToPack,
                         parameters.getEntryCMPLengthOffsetFactor(),
                         parameters.getEntryCMPOffset(),
                         parameters.getEntryCMPMinX(),
                         parameters.getEntryCMPMaxX(),
                         footPolygon,
                         previousFootPolygon,
                         supportSide);
   }

   private void computeCoPLocation(FramePoint2DBasics copLocationToPack,
                                   double lengthOffsetFactor,
                                   Vector2DReadOnly copOffset,
                                   double minXOffset,
                                   double maxXOffset,
                                   FrameConvexPolygon2DReadOnly basePolygon,
                                   FrameConvexPolygon2DReadOnly otherPolygon,
                                   RobotSide supportSide)
   {
      // FIXME this should be done in the sole frame, not the world frame
      copLocationToPack.setIncludingFrame(basePolygon.getCentroid());

      double copXOffset = MathTools.clamp(copOffset.getX() + lengthOffsetFactor * getStepLength(otherPolygon, basePolygon), minXOffset, maxXOffset);
      copLocationToPack.add(copXOffset, supportSide.negateIfLeftSide(copOffset.getY()));

      constrainToPolygon(copLocationToPack, basePolygon, parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrameAndProjectToXYPlane(worldFrame);
   }

   private final FramePoint2D mostForwardPointOnOtherPolygon = new FramePoint2D();

   private double getStepLength(FrameConvexPolygon2DReadOnly otherPolygon, FrameConvexPolygon2DReadOnly basePolygon)
   {
      mostForwardPointOnOtherPolygon.setIncludingFrame(otherPolygon.getVertex(EuclidGeometryPolygonTools.findVertexIndex(otherPolygon,
                                                                                                                         true,
                                                                                                                         Bound.MAX,
                                                                                                                         Bound.MAX)));
      mostForwardPointOnOtherPolygon.changeFrameAndProjectToXYPlane(basePolygon.getReferenceFrame());

      return mostForwardPointOnOtherPolygon.getX() - basePolygon.getMaxX();
   }

   /**
    * Constrains the specified CoP point to a safe distance within the specified support polygon by
    * projection
    */
   private void constrainToPolygon(FramePoint2DBasics copPointToConstrain,
                                   FrameConvexPolygon2DReadOnly constraintPolygon,
                                   double safeDistanceFromSupportPolygonEdges)
   {
      // don't need to do anything if it's already inside
      if (constraintPolygon.signedDistance(copPointToConstrain) <= -safeDistanceFromSupportPolygonEdges)
         return;

      polygonScaler.scaleConvexPolygon(constraintPolygon, safeDistanceFromSupportPolygonEdges, tempPolygon);
      copPointToConstrain.changeFrame(constraintPolygon.getReferenceFrame());
      tempPolygon.orthogonalProjection(copPointToConstrain);
   }
}