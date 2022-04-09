package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicStateInspector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   /**
    * If the ICP is this distance based the heel of the elading foot, toe off should happen, regardless of any of the other conditions.
    **/
   private final YoDouble distanceForwardFromHeel = new YoDouble("distanceForwardFromHeel", registry);

   /**
    * This checks to make sure the ICP isn't falling to the outside of the trailing foot.
    **/
   private final YoDouble minLateralDistance = new YoDouble("minLateralDistance", registry);

   /**
    * These variables make sure the ICP is far enough from the toe off point. If they're far enough, then there's probably enough control
    * authority to control them
    */
   private final YoDouble minDistanceFromTheToe = new YoDouble("minDistanceFromTheToe", registry);
   private final YoDouble minFractionOfStrideFromTheToe = new YoDouble("minFractionOfStrideFromTheToe", registry);

   private final YoDouble minDistanceFromOutsideEdge = new YoDouble("minDistanceFromOutsideEdge", registry);
   private final YoDouble minOrthgonalDistanceFromOutsideEdge = new YoDouble("minOrthogonalDistanceFromOutsideEdge", registry);
   private final YoDouble minDistanceFromInsideEdge = new YoDouble("minDistanceFromInsideEdge", registry);
   private final YoDouble minOrthgonalDistanceFromInsideEdge = new YoDouble("minOrthogonalDistanceFromInsideEdge", registry);

   private final YoBoolean currentIcpIsPastTheHeel = new YoBoolean("CurrentICPIsPastTheHeel", registry);
   private final YoBoolean currentIcpIsFarEnoughFromTheToe = new YoBoolean("currentIcpIsFarEnoughFromTheToe", registry);
   private final YoBoolean desiredIcpIsFarEnoughFromTheToe = new YoBoolean("desiredIcpIsFarEnoughFromTheToe", registry);
   private final YoBoolean currentIcpIsFarEnoughInside = new YoBoolean("currentIcpIsFarEnoughInside", registry);
   private final YoBoolean desiredIcpIsFarEnoughInside = new YoBoolean("desiredIcpIsFarEnoughInside", registry);

   private final YoBoolean currentIcpIsFarEnoughInsideOutsideEdge = new YoBoolean("currentIcpIsFarEnoughInsideOutsideEdge", registry);
   private final YoBoolean desiredIcpIsFarEnoughInsideOutsideEdge = new YoBoolean("desiredIcpIsFarEnoughInsideOutsideEdge", registry);

   private final YoBoolean currentIcpIsFarEnoughInsideInsideEdge = new YoBoolean("currentIcpIsFarEnoughInsideInsideEdge", registry);
   private final YoBoolean desiredIcpIsFarEnoughInsideInsideEdge = new YoBoolean("desiredIcpIsFarEnoughInsideInsideEdge", registry);

   private final YoBoolean isDesiredICPOKForToeOff = new YoBoolean("isDesiredICPOKForToeOff", registry);
   private final YoBoolean isCurrentICPOKForToeOff = new YoBoolean("isCurrentICPOKForToeOff", registry);

   private final YoBoolean dynamicsAreOkForToeOff = new YoBoolean("dynamicsAreOKForToeOff", registry);

   private final FrameConvexPolygon2D leadingFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D trailingFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D onToesPolygon = new FrameConvexPolygon2D();

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D toeOffPoint = new FramePoint2D();

   private final PoseReferenceFrame leadingFootFrame = new PoseReferenceFrame("leadingFootFrame", worldFrame);
   private final ZUpFrame leadingFootZUpFrame = new ZUpFrame(leadingFootFrame, "leadingFootZUpFrame");

   private final FrameLine2D insideEdge = new FrameLine2D();
   private final FrameLine2D outsideEdge = new FrameLine2D();

   public DynamicStateInspector(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void setPolygons(FrameConvexPolygon2DReadOnly leadingFootPolygon,
                           FrameConvexPolygon2DReadOnly trailingFootPolygon,
                           FrameConvexPolygon2DReadOnly onToesPolygon)
   {
      this.leadingFootPolygon.setIncludingFrame(leadingFootPolygon);
      this.trailingFootPolygon.setIncludingFrame(trailingFootPolygon);
      this.onToesPolygon.setIncludingFrame(onToesPolygon);
   }



   public void checkICPLocations(RobotSide trailingFootSide,
                                 FramePose3DReadOnly leadingFootPose,
                                 FramePoint2DReadOnly desiredICP,
                                 FramePoint2DReadOnly currentICP,
                                 FramePoint2DReadOnly toeOffPoint)
   {
      leadingFootFrame.setPoseAndUpdate(leadingFootPose);
      leadingFootZUpFrame.update();

      this.desiredICP.setIncludingFrame(desiredICP);
      this.currentICP.setIncludingFrame(currentICP);
      this.toeOffPoint.setIncludingFrame(toeOffPoint);

      double pseudoStepLength = computeDistanceToLeadingFoot();

      // If the ICP is far enough past the heel, should probably toe off.
      checkIfICPIsSuperFarForward();
      // Check the IcP for not being too far towards the outside of the stance foot
      checkIfICPIsTooFarOutward(trailingFootSide);
      // Check to make sure the ICP is far enough from the toe, so that its control authority doesn't matter too much.
      checkIfICPIsFarEnoughFromTheToe(pseudoStepLength);
      checkICPDistanceFromEdges(trailingFootSide);

      boolean isDesiredICPOKForToeOff = desiredIcpIsFarEnoughFromTheToe.getBooleanValue() && desiredIcpIsFarEnoughInside.getValue()
                                        && desiredIcpIsFarEnoughInsideOutsideEdge.getBooleanValue() && desiredIcpIsFarEnoughInsideInsideEdge.getBooleanValue();
      boolean isCurrentICPOKForToeOff = currentIcpIsFarEnoughFromTheToe.getBooleanValue() && currentIcpIsFarEnoughInside.getValue()
                                        && currentIcpIsFarEnoughInsideOutsideEdge.getBooleanValue() && currentIcpIsFarEnoughInsideInsideEdge.getBooleanValue();

      this.isCurrentICPOKForToeOff.set(isCurrentICPOKForToeOff);
      this.isDesiredICPOKForToeOff.set(isDesiredICPOKForToeOff);

      if (currentIcpIsPastTheHeel.getValue())
         dynamicsAreOkForToeOff.set(true);
      else
         dynamicsAreOkForToeOff.set(isCurrentICPOKForToeOff && isDesiredICPOKForToeOff);
   }

   private void checkIfICPIsSuperFarForward()
   {
      currentICP.changeFrame(leadingFootZUpFrame);
      toeOffPoint.changeFrame(leadingFootZUpFrame);

      leadingFootPolygon.changeFrameAndProjectToXYPlane(leadingFootZUpFrame);

      double minXValue = Math.max(leadingFootPolygon.getMinX(), toeOffPoint.getX());

      currentIcpIsPastTheHeel.set(currentICP.getX() > minXValue + distanceForwardFromHeel.getValue());
   }

   private void checkIfICPIsTooFarOutward(RobotSide trailingFootSide)
   {
      leadingFootPolygon.changeFrameAndProjectToXYPlane(leadingFootFrame);
      currentICP.changeFrame(leadingFootZUpFrame);
      toeOffPoint.changeFrame(leadingFootZUpFrame);

      if (trailingFootSide == RobotSide.LEFT)
      {
         double maxY = Math.max(toeOffPoint.getY() - minLateralDistance.getValue(), leadingFootPolygon.getMaxY());
         currentIcpIsFarEnoughInside.set(maxY > currentICP.getY());
         desiredIcpIsFarEnoughInside.set(maxY > desiredICP.getY());
      }
      else
      {
         double minY = Math.min(toeOffPoint.getY() + minLateralDistance.getValue(), leadingFootPolygon.getMinY());
         currentIcpIsFarEnoughInside.set(currentICP.getY() > minY);
         desiredIcpIsFarEnoughInside.set(desiredICP.getY() > minY);
      }
   }

   private void checkIfICPIsFarEnoughFromTheToe(double pseudoStepLength)
   {
      currentICP.changeFrame(worldFrame);
      desiredICP.changeFrame(worldFrame);
      toeOffPoint.changeFrame(worldFrame);

      double minDistance = Math.max(minDistanceFromTheToe.getDoubleValue(), minFractionOfStrideFromTheToe.getDoubleValue() * pseudoStepLength);
      double minDistanceSquared = MathTools.square(minDistance);

      currentIcpIsFarEnoughFromTheToe.set(currentICP.distanceSquared(toeOffPoint) > minDistanceSquared);
      desiredIcpIsFarEnoughFromTheToe.set(desiredICP.distanceSquared(toeOffPoint) > minDistanceSquared);
   }

   private final FrameVector2D errorDirection = new FrameVector2D();
   private final FramePoint2D projectedPoint = new FramePoint2D();
   private final FrameVector2D orthogonalDirection = new FrameVector2D();

   private void checkICPDistanceFromEdges(RobotSide trailingFootSide)
   {
      leadingFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      toeOffPoint.changeFrameAndProjectToXYPlane(worldFrame);
      currentICP.changeFrameAndProjectToXYPlane(worldFrame);
      desiredICP.changeFrameAndProjectToXYPlane(worldFrame);

      computeEdgesOfToeOff(trailingFootSide);

      errorDirection.sub(currentICP, desiredICP);
      errorDirection.normalize();

      checkOutsideEdge(trailingFootSide);
      checkInsideEdge(trailingFootSide);
   }

   private void computeEdgesOfToeOff(RobotSide trailingFootSide)
   {
      int startIndex = leadingFootPolygon.lineOfSightStartIndex(this.toeOffPoint);
      int endIndex = leadingFootPolygon.lineOfSightEndIndex(this.toeOffPoint);
      boolean isClockwise = leadingFootPolygon.isClockwiseOrdered();

      if (isClockwise == (trailingFootSide == RobotSide.RIGHT))
      {
         outsideEdge.setIncludingFrame(toeOffPoint, leadingFootPolygon.getVertex(endIndex));
         insideEdge.setIncludingFrame(toeOffPoint, leadingFootPolygon.getVertex(startIndex));
      }
      else
      {
         outsideEdge.setIncludingFrame(toeOffPoint, leadingFootPolygon.getVertex(startIndex));
         insideEdge.setIncludingFrame(toeOffPoint, leadingFootPolygon.getVertex(endIndex));
      }
   }

   private void checkOutsideEdge(RobotSide trailingFootSide)
   {
      outsideEdge.orthogonalProjection(currentICP, projectedPoint);
      orthogonalDirection.sub(projectedPoint, currentICP);
      orthogonalDirection.normalize();

      double currentOrthogonalDistanceToOutsideEdge = projectedPoint.distance(currentICP);
      double desiredOrthogonalDistanceToOutsideEdge = outsideEdge.distance(desiredICP);
      // FIXME this shouldn't be absolute
      double directionToEdgeInError = Math.abs(currentOrthogonalDistanceToOutsideEdge / errorDirection.dot(orthogonalDirection));

      if (outsideEdge.isPointOnSideOfLine(currentICP, trailingFootSide == RobotSide.LEFT))
      {  // inside or outside
         currentOrthogonalDistanceToOutsideEdge = -currentOrthogonalDistanceToOutsideEdge;
         directionToEdgeInError = -directionToEdgeInError;
      }

      if (outsideEdge.isPointOnSideOfLine(desiredICP, trailingFootSide == RobotSide.LEFT))
         desiredOrthogonalDistanceToOutsideEdge = -desiredOrthogonalDistanceToOutsideEdge;

      desiredIcpIsFarEnoughInsideOutsideEdge.set(desiredOrthogonalDistanceToOutsideEdge < minOrthgonalDistanceFromOutsideEdge.getValue());
      currentIcpIsFarEnoughInsideOutsideEdge.set(currentOrthogonalDistanceToOutsideEdge < minOrthgonalDistanceFromOutsideEdge.getValue()
                                                 && directionToEdgeInError < minDistanceFromOutsideEdge.getValue());
   }

   private void checkInsideEdge(RobotSide trailingFootSide)
   {
      insideEdge.orthogonalProjection(currentICP, projectedPoint);
      orthogonalDirection.sub(projectedPoint, currentICP);
      orthogonalDirection.normalize();

      double currentOrthogonalDistanceToInsideEdge = projectedPoint.distance(currentICP);
      double desiredOrthogonalDistanceToInsideEdge = insideEdge.distance(desiredICP);
      // FIXME this shouldn't be absolute
      double directionToEdgeInError = Math.abs(currentOrthogonalDistanceToInsideEdge / errorDirection.dot(orthogonalDirection));

      if (insideEdge.isPointOnSideOfLine(currentICP, trailingFootSide == RobotSide.RIGHT))
      {  // inside or outside
         currentOrthogonalDistanceToInsideEdge = -currentOrthogonalDistanceToInsideEdge;
         directionToEdgeInError = -directionToEdgeInError;
      }

      if (insideEdge.isPointOnSideOfLine(desiredICP, trailingFootSide == RobotSide.RIGHT))
         desiredOrthogonalDistanceToInsideEdge = -desiredOrthogonalDistanceToInsideEdge;

      desiredIcpIsFarEnoughInsideInsideEdge.set(desiredOrthogonalDistanceToInsideEdge < minOrthgonalDistanceFromInsideEdge.getValue());
      currentIcpIsFarEnoughInsideInsideEdge.set(currentOrthogonalDistanceToInsideEdge < minOrthgonalDistanceFromInsideEdge.getValue()
                                                && directionToEdgeInError < minDistanceFromInsideEdge.getValue());
   }

   private double computeDistanceToLeadingFoot()
   {
      this.toeOffPoint.changeFrameAndProjectToXYPlane(leadingFootZUpFrame);

      return toeOffPoint.distanceFromOrigin();
   }

   public boolean areDynamicsOkForToeOff()
   {
      return dynamicsAreOkForToeOff.getValue();
   }

   boolean isCurrentICPIsPastTheHeel()
   {
      return currentIcpIsPastTheHeel.getBooleanValue();
   }

   boolean isCurrentICPFarEnoughFromTheToe()
   {
      return currentIcpIsFarEnoughFromTheToe.getBooleanValue();
   }

   boolean isDesiredICPFarEnoughFromTheToe()
   {
      return desiredIcpIsFarEnoughFromTheToe.getBooleanValue();
   }

   boolean isCurrentICPFarEnoughInside()
   {
      return currentIcpIsFarEnoughInside.getBooleanValue();
   }

   boolean isDesiredICPFarEnoughInside()
   {
      return desiredIcpIsFarEnoughInside.getBooleanValue();
   }

   public void attachParameterChangeListener(YoVariableChangedListener changedListener)
   {
      distanceForwardFromHeel.addListener(changedListener);
      minLateralDistance.addListener(changedListener);
      minDistanceFromTheToe.addListener(changedListener);
      minFractionOfStrideFromTheToe.addListener(changedListener);
      minDistanceFromOutsideEdge.addListener(changedListener);
      minOrthgonalDistanceFromOutsideEdge.addListener(changedListener);
      minDistanceFromInsideEdge.addListener(changedListener);
      minOrthgonalDistanceFromInsideEdge.addListener(changedListener);
   }
}
