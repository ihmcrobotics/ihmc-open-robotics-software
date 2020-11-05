package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FlamingoCoPTrajectoryGenerator extends CoPTrajectoryGenerator
{
   private final CoPTrajectoryParameters parameters;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry;

   private final SideDependentList<FrameConvexPolygon2D> movingPolygonsInSole = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final SideDependentList<PoseReferenceFrame> stepFrames = new SideDependentList<>();

   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);

   private final FramePose3D tempPose = new FramePose3D();

   private final FrameConvexPolygon2D tempPolygon = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();
   private final FramePoint2D tempPointForCoPCalculation = new FramePoint2D();
   private final FramePoint2D midfootCoP = new FramePoint2D();

   private CoPPointViewer viewer = null;

   public FlamingoCoPTrajectoryGenerator(CoPTrajectoryParameters parameters, YoRegistry parentRegistry)
   {
      super(FlamingoCoPTrajectoryGenerator.class, parentRegistry);

      this.parameters = parameters;

      registry = new YoRegistry(getClass().getSimpleName());

      for (RobotSide robotSide : RobotSide.values)
      {
         stepFrames.put(robotSide, new PoseReferenceFrame(robotSide.getLowerCaseName() + "StepFrame", ReferenceFrame.getWorldFrame()));
      }

      parentRegistry.addChild(registry);
      clear();
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public void setWaypointViewer(CoPPointViewer viewer)
   {
      this.viewer = viewer;
   }

   public void clear()
   {
      contactStateProviders.clear();
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

      DynamicPlanningFootstep footstep = state.getFootstep(0);
      PlanningTiming timings = state.getTiming(0);
      RobotSide supportSide = footstep.getRobotSide().getOppositeSide();

      computeEntryCoPPointLocation(tempPointForCoPCalculation, movingPolygonsInSole.get(supportSide), supportSide);
      midfootCoP.interpolate(state.getInitialCoP(), tempPointForCoPCalculation, parameters.getDefaultTransferWeightDistribution());

      contactStateProvider.setDuration(parameters.getDefaultTransferSplitFraction() * timings.getTransferTime());
      contactStateProvider.setEndCopPosition(midfootCoP);

      SettableContactStateProvider previousContactState = contactStateProvider;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);
      contactStateProvider.setDuration((1.0 - parameters.getDefaultTransferSplitFraction()) * timings.getTransferTime());
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);

      previousContactState = contactStateProvider;
      contactStateProvider = contactStateProviders.add();
      contactStateProvider.setStartFromEnd(previousContactState);
      contactStateProvider.setDuration(Double.POSITIVE_INFINITY);
      tempPointForCoPCalculation.setIncludingFrame(movingPolygonsInSole.get(supportSide).getCentroid());
      tempPointForCoPCalculation.changeFrameAndProjectToXYPlane(worldFrame);
      contactStateProvider.setEndCopPosition(tempPointForCoPCalculation);

      if (viewer != null)
         viewer.updateWaypoints(contactStateProviders);
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }

   private void computeEntryCoPPointLocation(FramePoint2DBasics copLocationToPack,
                                             FrameConvexPolygon2DReadOnly supportPolygon,
                                             RobotSide supportSide)
   {
      copLocationToPack.setIncludingFrame(supportPolygon.getCentroid());

      Vector2DReadOnly copOffset = parameters.getEntryCMPOffset();
      double copXOffset = MathTools.clamp(copOffset.getX(), parameters.getEntryCMPMinX(), parameters.getEntryCMPMaxX());
      copLocationToPack.add(copXOffset, supportSide.negateIfLeftSide(copOffset.getY()));

      constrainToPolygon(copLocationToPack, supportPolygon, parameters.getMinimumDistanceInsidePolygon());
      copLocationToPack.changeFrameAndProjectToXYPlane(worldFrame);
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