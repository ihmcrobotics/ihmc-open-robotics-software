package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicStateInspector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble distanceSquaredOfCurrentICPFromToe = new YoDouble("distSqCurrentICPFromToe", registry);
   private final YoDouble distanceSquaredOfDesiredICPFromToe = new YoDouble("distSqDesiredICPFromToe", registry);
   private final YoDouble currentICPLateralDistanceInside = new YoDouble("currentICPLatDistInside", registry);
   private final YoDouble desiredICPLateralDistanceInside = new YoDouble("desiredICPLatDistInside", registry);

   private final YoDouble currentOrthogonalDistanceToOutsideEdge = new YoDouble("currentOrthoDistToOutsideTOEdge", registry);
   private final YoDouble desiredOrthogonalDistanceToOutsideEdge = new YoDouble("desiredOrthoDistToOutsideTOEdge", registry);
   private final YoDouble distanceAlongErrorToOutsideEdge = new YoDouble("distAlongErrorToOutTOEdge", registry);
   private final YoDouble normDistanceAlongErrorToOutsideEdge = new YoDouble("normDistAlongErrorToOutTOEdge", registry);

   private final YoDouble currentOrthogonalDistanceToInsideEdge = new YoDouble("currentOrthoDistToInTOEdge", registry);
   private final YoDouble desiredOrthogonalDistanceToInsideEdge = new YoDouble("desiredOrthoDistToInTOEdge", registry);
   private final YoDouble distanceAlongErrorToInsideEdge = new YoDouble("distAlongErrorToInTOEdge", registry);
   private final YoDouble normDistanceAlongErrorToInsideEdge = new YoDouble("normDistAlongErrorToInTOEdge", registry);

   private final YoDouble distanceAlongErrorToFullSupport = new YoDouble("distAlongErrorToFS", registry);
   private final YoDouble normDistanceAlongErrorToFullSupport = new YoDouble("normDistAlongErrorToFS", registry);

   private final YoBoolean currentIcpIsFarEnoughFromTheToe = new YoBoolean("currentIcpFarEnoughFromToe", registry);
   private final YoBoolean desiredIcpIsFarEnoughFromTheToe = new YoBoolean("desiredIcpFarEnoughFromToe", registry);
   private final YoBoolean currentIcpIsFarEnoughInside = new YoBoolean("currentIcpFarEnoughInside", registry);
   private final YoBoolean desiredIcpIsFarEnoughInside = new YoBoolean("desiredIcpFarEnoughInside", registry);

   private final YoBoolean currentIcpIsFarEnoughInsideOutsideEdge = new YoBoolean("currentIcpFarEnoughInsideOutEdge", registry);
   private final YoBoolean desiredIcpIsFarEnoughInsideOutsideEdge = new YoBoolean("desiredIcpFarEnoughInsideOutEdge", registry);

   private final YoBoolean currentIcpIsFarEnoughInsideInsideEdge = new YoBoolean("currentIcpFarEnoughInsideInEdge", registry);
   private final YoBoolean desiredIcpIsFarEnoughInsideInsideEdge = new YoBoolean("desiredIcpFarEnoughInsideInEdge", registry);

   private final YoDouble controlRatioInsideEdge = new YoDouble("controlRatioInEdge", registry);
   private final YoDouble controlRatioOutsideEdge = new YoDouble("controlRatioOutEdge", registry);
   private final YoBoolean toeingOffLosesTooMuchControl = new YoBoolean("toeOffLosesTooMuchControl", registry);

   private final YoBoolean isDesiredICPOKForToeOff = new YoBoolean("isDesiredICPOKForToeOff", registry);
   private final YoBoolean isCurrentICPOKForToeOff = new YoBoolean("isCurrentICPOKForToeOff", registry);

   private final GlitchFilteredYoBoolean dynamicsAreOkForToeOff = new GlitchFilteredYoBoolean("dynamicsAreOKForToeOff", registry, 4);
   private final GlitchFilteredYoBoolean dynamicsAreDefinitelyNotOKForToeOff = new GlitchFilteredYoBoolean("dynamicsAreDefinitelyNotOKForToeOff", registry, 4);

   private final FrameConvexPolygon2D leadingFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D trailingFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D onToesPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();

   private final FramePoint2D desiredICP = new FramePoint2D();
   private final FramePoint2D currentICP = new FramePoint2D();
   private final FramePoint2D toeOffPoint = new FramePoint2D();

   private final PoseReferenceFrame leadingFootFrame = new PoseReferenceFrame("leadingFootFrame", worldFrame);
   private final ZUpFrame leadingFootZUpFrame = new ZUpFrame(leadingFootFrame, "leadingFootZUpFrame");

   final FrameLine2D insideEdge = new FrameLine2D();
   final FrameLine2D outsideEdge = new FrameLine2D();
   private final FrameVector2D errorDirection = new FrameVector2D();

   final FramePoint2D pointOnInsideEdge = new FramePoint2D();
   final FramePoint2D pointOnOutsideEdge = new FramePoint2D();

   private final Point2DBasics tempPoint1 = new Point2D();
   private final Point2DBasics tempPoint2 = new Point2D();

   public DynamicStateInspector(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      currentIcpIsFarEnoughFromTheToe.set(false);
      desiredIcpIsFarEnoughFromTheToe.set(false);
      currentIcpIsFarEnoughInside.set(false);
      desiredIcpIsFarEnoughInside.set(false);

      currentIcpIsFarEnoughInsideOutsideEdge.set(false);
      desiredIcpIsFarEnoughInsideOutsideEdge.set(false);

      currentIcpIsFarEnoughInsideInsideEdge.set(false);
      desiredIcpIsFarEnoughInsideInsideEdge.set(false);

      toeingOffLosesTooMuchControl.set(false);

      isDesiredICPOKForToeOff.set(false);
      isCurrentICPOKForToeOff.set(false);

      dynamicsAreOkForToeOff.set(false);
      dynamicsAreDefinitelyNotOKForToeOff.set(true);
   }

   public void setPolygons(FrameConvexPolygon2DReadOnly leadingFootPolygon,
                           FrameConvexPolygon2DReadOnly trailingFootPolygon,
                           FrameConvexPolygon2DReadOnly onToesPolygon)
   {
      this.leadingFootPolygon.setIncludingFrame(leadingFootPolygon);
      this.trailingFootPolygon.setIncludingFrame(trailingFootPolygon);
      this.onToesPolygon.setIncludingFrame(onToesPolygon);

      supportPolygon.clear(worldFrame);
      supportPolygon.addVertices(leadingFootPolygon);
      supportPolygon.addVertices(trailingFootPolygon);
      supportPolygon.update();
   }

   public void checkICPLocations(DynamicStateInspectorParameters parameters,
                                 RobotSide trailingFootSide,
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

      // Check the IcP for not being too far towards the outside of the stance foot
      checkIfICPIsTooFarOutward(parameters, trailingFootSide);
      // Check to make sure the ICP is far enough from the toe, so that its control authority doesn't matter too much.
      checkIfICPIsFarEnoughFromTheToe(parameters, pseudoStepLength);
      checkICPDistanceFromEdges(parameters, trailingFootSide);

      boolean isDesiredICPOKForToeOff = desiredIcpIsFarEnoughFromTheToe.getBooleanValue() && desiredIcpIsFarEnoughInside.getValue()
                                        && desiredIcpIsFarEnoughInsideOutsideEdge.getBooleanValue() && desiredIcpIsFarEnoughInsideInsideEdge.getBooleanValue();
      boolean isCurrentICPOKForToeOff = currentIcpIsFarEnoughFromTheToe.getBooleanValue() && currentIcpIsFarEnoughInside.getValue()
                                        && currentIcpIsFarEnoughInsideOutsideEdge.getBooleanValue() && currentIcpIsFarEnoughInsideInsideEdge.getBooleanValue()
                                        && !toeingOffLosesTooMuchControl.getBooleanValue();

      this.isCurrentICPOKForToeOff.set(isCurrentICPOKForToeOff);
      this.isDesiredICPOKForToeOff.set(isDesiredICPOKForToeOff);

      dynamicsAreOkForToeOff.update(isCurrentICPOKForToeOff && isDesiredICPOKForToeOff);

      if (supportPolygon.isPointInside(currentICP) && supportPolygon.isPointInside(desiredICP))
      {
         dynamicsAreDefinitelyNotOKForToeOff.update(!dynamicsAreOkForToeOff.getBooleanValue());
      }
      else
      {
         dynamicsAreDefinitelyNotOKForToeOff.update(
               currentOrthogonalDistanceToInsideEdge.getDoubleValue() < 0.0 && desiredOrthogonalDistanceToInsideEdge.getDoubleValue() < 0.0);
      }
   }

   private void checkIfICPIsTooFarOutward(DynamicStateInspectorParameters parameters, RobotSide trailingFootSide)
   {
      leadingFootPolygon.changeFrame(leadingFootZUpFrame);
      currentICP.changeFrame(leadingFootZUpFrame);
      desiredICP.changeFrame(leadingFootZUpFrame);
      toeOffPoint.changeFrame(leadingFootZUpFrame);

      if (trailingFootSide == RobotSide.LEFT)
      {
         double maxY = Math.max(toeOffPoint.getY(), leadingFootPolygon.getMaxY() + parameters.getMinLateralDistanceInside());

         currentICPLateralDistanceInside.set(maxY - currentICP.getY());
         desiredICPLateralDistanceInside.set(maxY - desiredICP.getY());
      }
      else
      {
         double minY = Math.min(toeOffPoint.getY(), leadingFootPolygon.getMinY() - parameters.getMinLateralDistanceInside());

         currentICPLateralDistanceInside.set(currentICP.getY() - minY);
         desiredICPLateralDistanceInside.set(desiredICP.getY() - minY);
      }

      currentIcpIsFarEnoughInside.set(currentICPLateralDistanceInside.getValue() > parameters.getMinLateralDistanceInside());
      desiredIcpIsFarEnoughInside.set(desiredICPLateralDistanceInside.getValue() > parameters.getMinLateralDistanceInside());
   }

   private void checkIfICPIsFarEnoughFromTheToe(DynamicStateInspectorParameters parameters, double pseudoStepLength)
   {
      currentICP.changeFrame(worldFrame);
      desiredICP.changeFrame(worldFrame);
      toeOffPoint.changeFrame(worldFrame);

      double minDistance = Math.max(parameters.getMinDistanceFromTheToe(), parameters.getMinFractionOfStrideFromTheToe() * pseudoStepLength);
      double minDistanceSquared = MathTools.square(minDistance);

      distanceSquaredOfCurrentICPFromToe.set(currentICP.distanceSquared(toeOffPoint));
      distanceSquaredOfDesiredICPFromToe.set(desiredICP.distanceSquared(toeOffPoint));

      currentIcpIsFarEnoughFromTheToe.set(distanceSquaredOfCurrentICPFromToe.getValue() > minDistanceSquared);
      desiredIcpIsFarEnoughFromTheToe.set(distanceSquaredOfDesiredICPFromToe.getValue() > minDistanceSquared);
   }

   private void checkICPDistanceFromEdges(DynamicStateInspectorParameters parameters, RobotSide trailingFootSide)
   {
      leadingFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      toeOffPoint.changeFrameAndProjectToXYPlane(worldFrame);
      currentICP.changeFrameAndProjectToXYPlane(worldFrame);
      desiredICP.changeFrameAndProjectToXYPlane(worldFrame);

      computeEdgesOfToeOff(trailingFootSide);

      errorDirection.sub(currentICP, desiredICP);
      double errorMagnitude = errorDirection.length();
      errorDirection.normalize();

      checkOutsideEdge(parameters, trailingFootSide, errorMagnitude);
      checkInsideEdge(parameters, trailingFootSide, errorMagnitude);
      checkFullSupportPolygon(parameters, errorMagnitude);
   }

   private void computeEdgesOfToeOff(RobotSide trailingFootSide)
   {
      if (leadingFootPolygon.isPointInside(toeOffPoint))
      {
         // This is a bad situation, as it means that the robot will have overlapping footholds. However, it should be handled elsewhere in the controller.
         leadingFootPolygon.changeFrame(leadingFootZUpFrame);
         toeOffPoint.changeFrame(leadingFootZUpFrame);

         outsideEdge.setReferenceFrame(leadingFootZUpFrame);
         insideEdge.setReferenceFrame(leadingFootZUpFrame);
         outsideEdge.getPoint().set(toeOffPoint);
         insideEdge.getPoint().set(toeOffPoint);

         outsideEdge.getDirection().setX(toeOffPoint.getX() - leadingFootPolygon.getMinX());
         insideEdge.getDirection().setX(toeOffPoint.getX() - leadingFootPolygon.getMinX());

         if (trailingFootSide == RobotSide.RIGHT)
         {
            outsideEdge.getDirection().setY(leadingFootPolygon.getMaxY() - toeOffPoint.getY());
            insideEdge.getDirection().setY(leadingFootPolygon.getMinY() - toeOffPoint.getY());
         }
         else
         {
            insideEdge.getDirection().setY(leadingFootPolygon.getMaxY() - toeOffPoint.getY());
            outsideEdge.getDirection().setY(leadingFootPolygon.getMinY() - toeOffPoint.getY());
         }

         leadingFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
         outsideEdge.changeFrameAndProjectToXYPlane(worldFrame);
         insideEdge.changeFrameAndProjectToXYPlane(worldFrame);
         toeOffPoint.changeFrameAndProjectToXYPlane(worldFrame);
      }
      else
      {
         int startIndex = leadingFootPolygon.lineOfSightStartIndex(toeOffPoint);
         int endIndex = leadingFootPolygon.lineOfSightEndIndex(toeOffPoint);
         boolean isClockwise = leadingFootPolygon.isClockwiseOrdered();
         toeOffPoint.checkReferenceFrameMatch(leadingFootPolygon);

         if (startIndex == -1 || endIndex == -1)
            throw new RuntimeException("This should not happen.");

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
   }

   private void checkOutsideEdge(DynamicStateInspectorParameters parameters, RobotSide trailingFootSide, double errorMagnitude)
   {
      double currentOrthogonalDistanceToOutsideEdge = outsideEdge.distance(currentICP);
      double desiredOrthogonalDistanceToOutsideEdge = outsideEdge.distance(desiredICP);
      double distanceAlongErrorToOutsideEdge = rayDistance(outsideEdge, pointOnOutsideEdge);

      if (outsideEdge.isPointOnSideOfLine(currentICP, trailingFootSide == RobotSide.LEFT))
      {  // inside or outside
         currentOrthogonalDistanceToOutsideEdge = -currentOrthogonalDistanceToOutsideEdge;
         distanceAlongErrorToOutsideEdge = -distanceAlongErrorToOutsideEdge;
      }

      if (outsideEdge.isPointOnSideOfLine(desiredICP, trailingFootSide == RobotSide.LEFT))
         desiredOrthogonalDistanceToOutsideEdge = -desiredOrthogonalDistanceToOutsideEdge;

      this.desiredOrthogonalDistanceToOutsideEdge.set(desiredOrthogonalDistanceToOutsideEdge);
      this.currentOrthogonalDistanceToOutsideEdge.set(currentOrthogonalDistanceToOutsideEdge);
      this.distanceAlongErrorToOutsideEdge.set(distanceAlongErrorToOutsideEdge);

      normDistanceAlongErrorToOutsideEdge.set(this.distanceAlongErrorToOutsideEdge.getValue() / errorMagnitude);

      double minDistanceAlongEdge = parameters.getMinDistanceAlongErrorFromOutsideEdge();
      if (Double.isFinite(parameters.getMinNormalizedDistanceFromOutsideEdge()))
         minDistanceAlongEdge = Math.min(minDistanceAlongEdge, -parameters.getMinNormalizedDistanceFromOutsideEdge() * errorMagnitude);

      desiredIcpIsFarEnoughInsideOutsideEdge.set(desiredOrthogonalDistanceToOutsideEdge < parameters.getMinOrthogonalDistanceFromOutsideEdge());
      currentIcpIsFarEnoughInsideOutsideEdge.set(currentOrthogonalDistanceToOutsideEdge < parameters.getMinOrthogonalDistanceFromOutsideEdge()
                                                 && distanceAlongErrorToOutsideEdge < minDistanceAlongEdge);
   }

   private void checkInsideEdge(DynamicStateInspectorParameters parameters, RobotSide trailingFootSide, double errorMagnitude)
   {
      double currentOrthogonalDistanceToInsideEdge = insideEdge.distance(currentICP);
      double desiredOrthogonalDistanceToInsideEdge = insideEdge.distance(desiredICP);
      double distanceAlongErrorToInsideEdge = rayDistance(insideEdge, pointOnInsideEdge);

      if (insideEdge.isPointOnSideOfLine(currentICP, trailingFootSide == RobotSide.RIGHT))
      {  // inside or outside
         currentOrthogonalDistanceToInsideEdge = -currentOrthogonalDistanceToInsideEdge;
         distanceAlongErrorToInsideEdge = -distanceAlongErrorToInsideEdge;
      }

      if (insideEdge.isPointOnSideOfLine(desiredICP, trailingFootSide == RobotSide.RIGHT))
         desiredOrthogonalDistanceToInsideEdge = -desiredOrthogonalDistanceToInsideEdge;

      this.desiredOrthogonalDistanceToInsideEdge.set(desiredOrthogonalDistanceToInsideEdge);
      this.currentOrthogonalDistanceToInsideEdge.set(currentOrthogonalDistanceToInsideEdge);
      this.distanceAlongErrorToInsideEdge.set(distanceAlongErrorToInsideEdge);

      normDistanceAlongErrorToInsideEdge.set(this.distanceAlongErrorToInsideEdge.getValue() / errorMagnitude);

      boolean currentIsFarEnoughInside = currentOrthogonalDistanceToInsideEdge < parameters.getMinOrthogonalDistanceFromInsideEdge();
      double minDistanceFromEdge = parameters.getMinDistanceAlongErrorFromInsideEdge();
      if (Double.isFinite(parameters.getMinNormalizedDistanceFromInsideEdge()))
         minDistanceFromEdge = Math.min(minDistanceFromEdge, -parameters.getMinNormalizedDistanceFromInsideEdge() * errorMagnitude);
      boolean dynamicsFarEnoughInside = distanceAlongErrorToInsideEdge < minDistanceFromEdge;

      desiredIcpIsFarEnoughInsideInsideEdge.set(desiredOrthogonalDistanceToInsideEdge < parameters.getMinOrthogonalDistanceFromInsideEdge());
      currentIcpIsFarEnoughInsideInsideEdge.set(currentIsFarEnoughInside && dynamicsFarEnoughInside);
   }

   private void checkFullSupportPolygon(DynamicStateInspectorParameters parameters, double errorMagnitude)
   {
      if (Double.isFinite(parameters.getMaxRatioOfControlDecreaseFromToeingOff()) && supportPolygon.isPointInside(desiredICP) && supportPolygon.isPointInside(
            currentICP))
      {
         if (normDistanceAlongErrorToInsideEdge.getDoubleValue() > 0.0 || normDistanceAlongErrorToOutsideEdge.getDoubleValue() > 0.0)
         {
            toeingOffLosesTooMuchControl.set(true);
         }
         else
         {
            int intersections = EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(desiredICP,
                                                                                                      errorDirection,
                                                                                                      supportPolygon.getVertexBufferView(),
                                                                                                      supportPolygon.getNumberOfVertices(),
                                                                                                      supportPolygon.isClockwiseOrdered(),
                                                                                                      tempPoint1,
                                                                                                      tempPoint2);

            distanceAlongErrorToFullSupport.set(currentICP.distance(tempPoint1));
            normDistanceAlongErrorToFullSupport.set(distanceAlongErrorToFullSupport.getValue() / errorMagnitude);

            // the error distance is negative, so cancel that
            controlRatioInsideEdge.set(-normDistanceAlongErrorToFullSupport.getValue() / normDistanceAlongErrorToInsideEdge.getDoubleValue());
            controlRatioOutsideEdge.set(-normDistanceAlongErrorToFullSupport.getValue() / normDistanceAlongErrorToOutsideEdge.getDoubleValue());
            boolean insideEdgeWouldFail = controlRatioInsideEdge.getValue() > parameters.getMaxRatioOfControlDecreaseFromToeingOff();
            boolean outsideEdgeWouldFail = controlRatioOutsideEdge.getValue() > parameters.getMaxRatioOfControlDecreaseFromToeingOff();

            if (Double.isFinite(parameters.getMaxNormalizedErrorNeededForControl()))
            {
               insideEdgeWouldFail &= -normDistanceAlongErrorToInsideEdge.getDoubleValue() < parameters.getMaxNormalizedErrorNeededForControl();
               outsideEdgeWouldFail &= -normDistanceAlongErrorToOutsideEdge.getDoubleValue() < parameters.getMaxNormalizedErrorNeededForControl();
            }
            toeingOffLosesTooMuchControl.set(insideEdgeWouldFail || outsideEdgeWouldFail);
         }
      }
      else
      {
         toeingOffLosesTooMuchControl.set(false);
      }
   }

   double rayDistance(FrameLine2DReadOnly lineToIntersection, Point2DBasics intersectionToPack)
   {
      boolean success = EuclidCoreMissingTools.intersectionBetweenRay2DAndLine2D(desiredICP,
                                                                                 errorDirection,
                                                                                 lineToIntersection.getPoint(),
                                                                                 lineToIntersection.getDirection(),
                                                                                 intersectionToPack);

      if (success)
         return intersectionToPack.distance(currentICP);

      intersectionToPack.setToNaN();
      return Double.POSITIVE_INFINITY;
   }

   private double computeDistanceToLeadingFoot()
   {
      toeOffPoint.changeFrameAndProjectToXYPlane(leadingFootZUpFrame);
      return toeOffPoint.distanceFromOrigin();
   }

   public boolean areDynamicsDefinitelyNotOkForToeOff()
   {
      return dynamicsAreDefinitelyNotOKForToeOff.getBooleanValue();
   }

   public boolean areDynamicsOkForToeOff()
   {
      return dynamicsAreOkForToeOff.getValue();
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

   boolean isCurrentICPFarEnoughInsideOutsideEdge()
   {
      return currentIcpIsFarEnoughInsideOutsideEdge.getBooleanValue();
   }

   boolean isDesiredICPFarEnoughInsideOutsideEdge()
   {
      return desiredIcpIsFarEnoughInsideOutsideEdge.getBooleanValue();
   }

   boolean isCurrentICPFarEnoughInsideInsideEdge()
   {
      return currentIcpIsFarEnoughInsideInsideEdge.getBooleanValue();
   }

   boolean isDesiredICPFarEnoughInsideInsideEdge()
   {
      return desiredIcpIsFarEnoughInsideInsideEdge.getBooleanValue();
   }

   boolean isDesiredICPOKForToeOff()
   {
      return isDesiredICPOKForToeOff.getBooleanValue();
   }

   boolean isCurrentICPOKForToeOff()
   {
      return isCurrentICPOKForToeOff.getBooleanValue();
   }

   double getDistanceSquaredOfCurrentICPFromToe()
   {
      return distanceSquaredOfCurrentICPFromToe.getValue();
   }

   double getDistanceSquaredOfDesiredICPFromToe()
   {
      return distanceSquaredOfDesiredICPFromToe.getValue();
   }

   double getLateralDistanceOfCurrentICPInside()
   {
      return currentICPLateralDistanceInside.getDoubleValue();
   }

   double getLateralDistanceOfDesiredICPInside()
   {
      return desiredICPLateralDistanceInside.getDoubleValue();
   }

   double getCurrentOrthogonalDistanceToOutsideEdge()
   {
      return currentOrthogonalDistanceToOutsideEdge.getDoubleValue();
   }

   double getDesiredOrthogonalDistanceToOutsideEdge()
   {
      return desiredOrthogonalDistanceToOutsideEdge.getDoubleValue();
   }

   double getDistanceAlongErrorToOutsideEdge()
   {
      return distanceAlongErrorToOutsideEdge.getDoubleValue();
   }

   double getDistaneAlongErrorToFullSupport()
   {
      return distanceAlongErrorToFullSupport.getDoubleValue();
   }

   double getCurrentOrthogonalDistanceToInsideEdge()
   {
      return currentOrthogonalDistanceToInsideEdge.getDoubleValue();
   }

   double getDesiredOrthogonalDistanceToInsideEdge()
   {
      return desiredOrthogonalDistanceToInsideEdge.getDoubleValue();
   }

   double getDistanceAlongErrorToInsideEdge()
   {
      return distanceAlongErrorToInsideEdge.getDoubleValue();
   }

   double getNormalizedDistanceAlongErrorToOutsideEdge()
   {
      return normDistanceAlongErrorToOutsideEdge.getDoubleValue();
   }

   double getNormalizedDistanceAlongErrorToInsideEdge()
   {
      return normDistanceAlongErrorToInsideEdge.getDoubleValue();
   }

   double getNormalizedDistanceAlongErrorToFullSupport()
   {
      return normDistanceAlongErrorToFullSupport.getDoubleValue();
   }

   double getControlRatioInsideEdge()
   {
      return controlRatioInsideEdge.getDoubleValue();
   }

   double getControlRatioOutsideEdge()
   {
      return controlRatioOutsideEdge.getDoubleValue();
   }
}
