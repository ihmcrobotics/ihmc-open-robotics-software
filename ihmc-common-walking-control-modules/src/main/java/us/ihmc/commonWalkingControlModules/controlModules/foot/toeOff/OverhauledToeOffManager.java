package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ToeOffManager;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
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
   private Footstep nextNextFootstep;

   private final SideDependentList<FrameConvexPolygon2D> footDefaultPolygons;
   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;

   private final FrameConvexPolygon2D leadingFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D trailingFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D nextFootSupportPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D onToesSupportPolygon = new FrameConvexPolygon2D();

   private final ToeOffStepPositionInspector stepPositionInspector;
   private final DynamicStateInspector dynamicStateInspector;
   private final LegJointLimitsInspector jointLimitsInspector;

   private final DynamicStateInspectorParameters dynamicStateInspectorParameters;

   private final AbstractToeContact lineContactComputer = new ToeLineContact();
   private final AbstractToeContact pointContactComputer = new ToePointContact();

   private final ToeOffCalculator toeOffCalculator;

   private final BooleanProvider doToeOffIfPossibleInDoubleSupport;
   private final BooleanProvider doToeOffIfPossibleInSingleSupport;

   private final BooleanProvider useToeLineContactInSwing;
   private final BooleanProvider useToeLineContactInTransfer;

   private final YoBoolean doPointToeOff = new YoBoolean("doPointToeOff", registry);
   private final YoBoolean doLineToeOff = new YoBoolean("doLineToeOff", registry);

   private final FramePose3D nextFrontFootPose = new FramePose3D();

   public OverhauledToeOffManager(WalkingControllerParameters walkingControllerParameters,
                                  SideDependentList<MovingReferenceFrame> soleZUpFrames,
                                  ToeOffCalculator toeOffCalculator,
                                  YoRegistry parentRegistry)
   {
      this.toeOffCalculator = toeOffCalculator;
      this.soleZUpFrames = soleZUpFrames;

      ToeOffParameters toeOffParameters = walkingControllerParameters.getToeOffParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();

      double footLength = steppingParameters.getFootBackwardOffset() + steppingParameters.getFootForwardOffset();

      stepPositionInspector = new ToeOffStepPositionInspector(soleZUpFrames,
                                                              toeOffParameters,
                                                              steppingParameters.getInPlaceWidth(),
                                                              footLength,
                                                              parentRegistry);
      jointLimitsInspector = new LegJointLimitsInspector(toeOffParameters, parentRegistry);

      footDefaultPolygons = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         footDefaultPolygons.put(robotSide, new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(feet.get(robotSide).getContactPoints2d())));
      }

      useToeLineContactInSwing = new BooleanParameter("useToeLineContactInSwing", registry, toeOffParameters.useToeOffLineContactInSwing());
      useToeLineContactInTransfer = new BooleanParameter("useToeLineContactInTransfer", registry, toeOffParameters.useToeOffLineContactInTransfer());

      doToeOffIfPossibleInDoubleSupport = new BooleanParameter("doToeOffIfPossibleInDoubleSupport", registry, toeOffParameters.doToeOffIfPossible());
      doToeOffIfPossibleInSingleSupport = new BooleanParameter("doToeOffIfPossibleInSingleSupport", registry, toeOffParameters.doToeOffIfPossibleInSingleSupport());

      parentRegistry.addChild(registry);
   }


   public void submitNextFootstep(Footstep nextFootstep, Footstep nextNextFootstep)
   {
      this.nextFootstep = nextFootstep;
      this.nextNextFootstep = nextNextFootstep;
   }

   public void updateToeOffStatusSingleSupport(FramePoint3DReadOnly exitCMP,
                                               FramePoint2DReadOnly desiredECMP,
                                               FramePoint2DReadOnly desiredCoP,
                                               FramePoint2DReadOnly desiredICP,
                                               FramePoint2DReadOnly currentICP,
                                               FramePoint2DReadOnly finalDesiredICP)
   {
      setPolygonFromNextFootstep(nextFootSupportPolygon);

      RobotSide trailingLeg = nextFootstep.getRobotSide().getOppositeSide();

      FramePoint2D toeOffPoint;
      if (useToeLineContactInSwing.getValue())
      {
         lineContactComputer.updateToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, nextFootSupportPolygon);
         toeOffPoint = lineContactComputer.getToeOffPoint();
      }
      else
      {
         pointContactComputer.updateToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, nextFootSupportPolygon);
         toeOffPoint = pointContactComputer.getToeOffPoint();
      }

//      if (finalDesiredICP != null && !onToesSupportPolygon.isPointInside(finalDesiredICP))
//      { // This allows to better account for long and/or fast steps when the final ICP lies outside the toe-off support polygon.
//         onToesSupportPolygon.addVertex(finalDesiredICP);
//         onToesSupportPolygon.update();
//      }

      trailingFootSupportPolygon.clear(feet.get(trailingLeg).getSoleFrame());
      for (int i = 0; i < feet.get(trailingLeg).getTotalNumberOfContactPoints(); i++)
         trailingFootSupportPolygon.addVertex(feet.get(trailingLeg).getContactPoints2d().get(i));
      trailingFootSupportPolygon.update();
      trailingFootSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      updateToeOffConditions(trailingLeg,
                             desiredICP,
                             currentICP,
                             toeOffPoint,
                             doToeOffIfPossibleInSingleSupport.getValue(),
                             useToeLineContactInSwing.getValue(),
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
      setPolygonFromSupportFoot(trailingLeg, leadingFootSupportPolygon);

      FramePoint2D toeOffPoint;
      if (useToeLineContactInTransfer.getValue())
      {
         lineContactComputer.updateToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, nextFootSupportPolygon);
         toeOffPoint = lineContactComputer.getToeOffPoint();
      }
      else
      {
         pointContactComputer.updateToeSupportPolygon(exitCMP, desiredECMP, trailingLeg, nextFootSupportPolygon);
         toeOffPoint = pointContactComputer.getToeOffPoint();
      }

//      if (finalDesiredICP != null && !onToesSupportPolygon.isPointInside(finalDesiredICP))
//      { // This allows to better account for long and/or fast steps when the final ICP lies outside the toe-off support polygon.
//         onToesSupportPolygon.addVertex(finalDesiredICP);
//         onToesSupportPolygon.update();
//      }

      trailingFootSupportPolygon.clear(feet.get(trailingLeg).getSoleFrame());
      for (int i = 0; i < feet.get(trailingLeg).getTotalNumberOfContactPoints(); i++)
         trailingFootSupportPolygon.addVertex(feet.get(trailingLeg).getContactPoints2d().get(i));
      trailingFootSupportPolygon.update();
      trailingFootSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

      nextFrontFootPose.setToZero(soleZUpFrames.get(trailingLeg.getOppositeSide()));
      nextFrontFootPose.changeFrame(worldFrame);

      updateToeOffConditions(trailingLeg,
                             desiredICP,
                             currentICP,
                             toeOffPoint,
                             doToeOffIfPossibleInDoubleSupport.getValue(),
                             useToeLineContactInTransfer.getValue(),
                             nextFrontFootPose);
   }

   private void updateToeOffConditions(RobotSide trailingLeg,
                                       FramePoint2DReadOnly desiredICP,
                                       FramePoint2DReadOnly currentICP,
                                       FramePoint2DReadOnly toeOffPoint,
                                       boolean allowToeOff,
                                       boolean useToeLine,
                                       FramePose3DReadOnly nextFrontFootPose)
   {
      dynamicStateInspector.setPolygons(nextFootSupportPolygon,
                                        trailingFootSupportPolygon,
                                        onToesSupportPolygon);
      boolean wellPositioned = stepPositionInspector.isFrontFootWellPositionedForToeOff(trailingLeg, nextFrontFootPose);
      jointLimitsInspector.updateToeOffConditions(trailingLeg);
      dynamicStateInspector.checkICPLocations(dynamicStateInspectorParameters,
                                              trailingLeg,
                                              nextFrontFootPose,
                                              desiredICP,
                                              currentICP,
                                              toeOffPoint);

      boolean doToeOff = allowToeOff;
      doToeOff &= wellPositioned;
      doToeOff &= (jointLimitsInspector.needToSwitchToToeOffDueToJointLimit() || dynamicStateInspector.areDynamicsOkForToeOff());

      doPointToeOff.set(doToeOff && useToeLine);
      doLineToeOff.set(doToeOff && !useToeLine);
   }

   private void setPolygonFromSupportFoot(RobotSide trailingLeg, FrameConvexPolygon2DBasics polygonToPack)
   {
      RobotSide leadingLeg = trailingLeg.getOppositeSide();
      if (footContactStates != null && footContactStates.get(leadingLeg).getTotalNumberOfContactPoints() > 0)
      {
         footContactStates.get(leadingLeg).getContactFramePointsInContact(contactStatePoints);
         polygonToPack.clear(worldFrame);
         for (int i = 0; i < contactStatePoints.size(); i++)
            polygonToPack.addVertexMatchingFrame(contactStatePoints.get(i));
         polygonToPack.update();
      }
      else
      {
         polygonToPack.setIncludingFrame(footDefaultPolygons.get(leadingLeg));
         polygonToPack.changeFrameAndProjectToXYPlane(worldFrame);
      }
   }


   private boolean setPolygonFromNextFootstep(FrameConvexPolygon2DBasics polygonToPack)
   {
      if (nextFootstep == null || nextFootstep.getRobotSide() == null)
         return false;

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

      return true;
   }

   private abstract class AbstractToeContact
   {
      protected final FrameLineSegment2D toeOffLine = new FrameLineSegment2D();
      protected final FramePoint2D toeOffPoint = new FramePoint2D();
      protected final FramePoint2D tmpPoint2d = new FramePoint2D();

      public abstract void updateToeSupportPolygon(FramePoint3DReadOnly exitCMP,
                                                   FramePoint2DReadOnly desiredECMP,
                                                   RobotSide trailingSide,
                                                   FrameConvexPolygon2DReadOnly leadingSupportPolygon);

      protected void computeToeContacts(RobotSide supportSide)
      {
         FrameConvexPolygon2D footDefaultPolygon = footDefaultPolygons.get(supportSide);
         ReferenceFrame referenceFrame = footDefaultPolygon.getReferenceFrame();
         toeOffLine.getFirstEndpoint().set(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);
         toeOffLine.getSecondEndpoint().set(referenceFrame, Double.NEGATIVE_INFINITY, 0.0);

         // gets the leading two toe points
         for (int i = 0; i < footDefaultPolygon.getNumberOfVertices(); i++)
         {
            tmpPoint2d.setIncludingFrame(footDefaultPolygon.getVertex(i));
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

         toeOffPoint.setToZero(footDefaultPolygon.getReferenceFrame());
         toeOffLine.midpoint(toeOffPoint);
      }

      public FramePoint2D getToeOffPoint()
      {
         return toeOffPoint;
      }
   }

   private class ToeLineContact extends AbstractToeContact
   {
      @Override
      public void updateToeSupportPolygon(FramePoint3DReadOnly exitCMP,
                                          FramePoint2DReadOnly desiredECMP,
                                          RobotSide trailingSide,
                                          FrameConvexPolygon2DReadOnly leadingSupportPolygon)
      {
         if (exitCMP == null)
            computeToeContacts(trailingSide);
         else
            computeToeContacts(exitCMP, desiredECMP, trailingSide);

         onToesSupportPolygon.setIncludingFrame(leadingSupportPolygon);
         onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

         onToesSupportPolygon.addVertexMatchingFrame(toeOffLine.getFirstEndpoint(), false);
         onToesSupportPolygon.addVertexMatchingFrame(toeOffLine.getSecondEndpoint(), false);

         onToesSupportPolygon.update();

         toeOffLine.midpoint(toeOffPoint);
      }


      private void computeToeContacts(FramePoint3DReadOnly exitCMP, FramePoint2DReadOnly desiredECMP, RobotSide supportSide)
      {
         toeOffCalculator.setExitCMP(exitCMP, supportSide);
         toeOffCalculator.computeToeOffContactLine(desiredECMP, supportSide);

         toeOffLine.setToZero(soleZUpFrames.get(supportSide));
         toeOffCalculator.getToeOffContactLine(toeOffLine, supportSide);
      }


   }

   private class ToePointContact extends AbstractToeContact
   {
      @Override
      public void updateToeSupportPolygon(FramePoint3DReadOnly exitCMP,
                                          FramePoint2DReadOnly desiredECMP,
                                          RobotSide trailingSide,
                                          FrameConvexPolygon2DReadOnly leadingSupportPolygon)
      {
         if (exitCMP == null)
            computeToeContacts(trailingSide);
         else
            computeToeContacts(exitCMP, desiredECMP, trailingSide);

         onToesSupportPolygon.setIncludingFrame(leadingSupportPolygon);
         onToesSupportPolygon.changeFrameAndProjectToXYPlane(worldFrame);

         onToesSupportPolygon.addVertexMatchingFrame(toeOffPoint, false);
         onToesSupportPolygon.update();
      }


      private void computeToeContacts(FramePoint3DReadOnly exitCMP, FramePoint2DReadOnly desiredECMP, RobotSide supportSide)
      {
         toeOffCalculator.setExitCMP(exitCMP, supportSide);
         toeOffCalculator.computeToeOffContactPoint(desiredECMP, supportSide);

         toeOffPoint.setToZero(soleZUpFrames.get(supportSide));
         toeOffCalculator.getToeOffContactPoint(toeOffPoint, supportSide);
      }
   }
}
