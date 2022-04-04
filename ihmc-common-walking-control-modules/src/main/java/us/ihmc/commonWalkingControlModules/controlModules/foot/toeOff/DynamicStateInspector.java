package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicStateInspector
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble ecmpProximityToOnToes = new YoDouble("ecmpProximityToOnToes", registry);
   private final YoDouble copProximityToOnToes = new YoDouble("copProximityToOnToes", registry);
   private final YoDouble desiredICPDistanceToOnToes = new YoDouble("desiredICPProximityToOnToes", registry);
   private final YoDouble currentICPDistanceToOnToes = new YoDouble("currentICPProximityToOnToes", registry);
   private final YoDouble desiredICPProximityToLeadingFoot = new YoDouble("desiredICPProximityToLeadingFoot", registry);
   private final YoDouble currentICPProximityToLeadingFoot = new YoDouble("currentICPProximityToLeadingFoot", registry);

   private final YoBoolean isDesiredICPOKForToeOff = new YoBoolean("isDesiredICPOKForToeOff", registry);
   private final YoBoolean isCurrentICPOKForToeOff = new YoBoolean("isCurrentICPOKForToeOff", registry);
   private final YoBoolean isDesiredECMPOKForToeOff = new YoBoolean("isDesiredECMPOKForToeOff", registry);
   private final YoBoolean isDesiredCoPOKForToeOff = new YoBoolean("isDesiredCoPOKForToeOff", registry);
   private final YoBoolean isPerfectCoPOKForToeOff = new YoBoolean("isPerfectCoPOKForToeOff", registry);

   private static final int smallGlitchWindowSize = 2;

   private final GlitchFilteredYoBoolean isDesiredICPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredICPOKForToeOffFilt", registry,
                                                                                                   isDesiredICPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isCurrentICPOKForToeOffFilt = new GlitchFilteredYoBoolean("isCurrentICPOKForToeOffFilt", registry,
                                                                                                   isCurrentICPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isDesiredECMPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredECMPOKForToeOffFilt", registry,
                                                                                                    isDesiredECMPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isDesiredCoPOKForToeOffFilt = new GlitchFilteredYoBoolean("isDesiredCoPOKForToeOffFilt", registry,
                                                                                                   isDesiredCoPOKForToeOff, smallGlitchWindowSize);
   private final GlitchFilteredYoBoolean isPerfectCoPOKForToeOffFilt = new GlitchFilteredYoBoolean("isPerfectCoPOKForToeOffFilt", registry,
                                                                                                   isPerfectCoPOKForToeOff, smallGlitchWindowSize);

   private final SideDependentList<MovingReferenceFrame> soleZUpFrames;

   private final FrameConvexPolygon2D leadingFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D onToesPolygon = new FrameConvexPolygon2D();

   public DynamicStateInspector(SideDependentList<MovingReferenceFrame> soleZUpFrames, YoRegistry parentRegistry)
   {
      this.soleZUpFrames = soleZUpFrames;

      parentRegistry.addChild(registry);
   }

   public void updateForDynamicState(RobotSide trailingLeg, FramePose3DReadOnly frontFootPose)

   {

   }


   public void setPolygons(FrameConvexPolygon2DReadOnly leadingFootPolygon,
                           FrameConvexPolygon2DReadOnly onToesPolygon)
   {
      this.leadingFootPolygon.setIncludingFrame(leadingFootPolygon);
      this.onToesPolygon.setIncludingFrame(onToesPolygon);
   }


   public void checkICPLocations(RobotSide trailingLeg,
                                 FramePoint2DReadOnly desiredICP,
                                 FramePoint2DReadOnly currentICP,
                                 FramePoint2DReadOnly toeOffPoint,
                                 FramePoint3DReadOnly nextFootPosition,
                                 double percentProximity)
   {
      icpIsInsideSupportFoot.set(trailingFootSupportPolygon.isPointInside(currentICP));

      desiredICPDistanceToOnToes.set(onToesPolygon.signedDistance(desiredICP));
      currentICPDistanceToOnToes.set(onToesPolygon.signedDistance(currentICP));

      double distanceToLeadingFoot = computeDistanceToLeadingFoot(nextFootPosition, toeOffPoint);

      currentICPProximityToLeadingFoot.set(leadingFootPolygon.signedDistance(currentICP) / distanceToLeadingFoot);
      desiredICPProximityToLeadingFoot.set(leadingFootPolygon.signedDistance(desiredICP) / distanceToLeadingFoot);

      boolean isDesiredICPOKForToeOff = desiredICPDistanceToOnToes.getDoubleValue() < icpProximityForToeOff.getValue();
      isDesiredICPOKForToeOff &= desiredICPProximityToLeadingFoot.getDoubleValue() < percentProximity;

      boolean isCurrentICPOKForToeOff = currentICPDistanceToOnToes.getDoubleValue() < icpProximityForToeOff.getValue();
      isCurrentICPOKForToeOff &= currentICPProximityToLeadingFoot.getDoubleValue() < percentProximity;

      this.isCurrentICPOKForToeOff.set(isCurrentICPOKForToeOff);
      this.isDesiredICPOKForToeOff.set(isDesiredICPOKForToeOff);
      this.isCurrentICPOKForToeOffFilt.update();
      this.isDesiredICPOKForToeOffFilt.update();
   }

   private final FramePoint2D toeOffPoint = new FramePoint2D();
   private final FramePoint2D nextFootPosition = new FramePoint2D();

   private double computeDistanceToLeadingFoot(FramePoint3DReadOnly nextFootPosition,
                                               FramePoint2DReadOnly toeOffPoint)
   {
      this.toeOffPoint.setIncludingFrame(toeOffPoint);
      this.nextFootPosition.setIncludingFrame(nextFootPosition);

      this.toeOffPoint.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      this.nextFootPosition.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      return this.nextFootPosition.distance(this.toeOffPoint);
   }
}
