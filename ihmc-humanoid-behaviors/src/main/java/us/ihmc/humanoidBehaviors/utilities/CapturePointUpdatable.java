package us.ihmc.humanoidBehaviors.utilities;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class CapturePointUpdatable implements Updatable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoFramePoint2D yoDesiredCapturePoint = new YoFramePoint2D("desiredCapturePoint", worldFrame, registry);
   private final YoFramePoint2D yoCapturePoint = new YoFramePoint2D("capturePoint", worldFrame, registry);
   private final YoFrameConvexPolygon2D yoSupportPolygon = new YoFrameConvexPolygon2D("supportPolygon", "", worldFrame, 30, registry);
   private final SideDependentList<YoFrameConvexPolygon2D> yoFootSupportPolygons = new SideDependentList<>();
   private final YoEnum<RobotSide> yoSupportLeg = new YoEnum<>("supportLeg", registry, RobotSide.class, true);
   private final YoBoolean yoDoubleSupport = new YoBoolean("doubleSupport", registry);

   // Computed Stuff
   private final YoDouble icpError = new YoDouble("icpError", registry);
   private final YoDouble minIcpDistanceToSupportPolygon = new YoDouble("minIcpDistanceToSupportPolygon", registry);
   private final YoBoolean tippingDetected = new YoBoolean("tippingDetected", registry);
   private final double MAX_CAPTURE_POINT_ERROR_M = 0.5 * 0.075; // Reasonable value < 0.01   Max < 0.02

   private final FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
   private final SideDependentList<FrameConvexPolygon2D> footSupportPolygons = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber;

   public CapturePointUpdatable(CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoRegistry parentRegistry)
   {
      this.capturabilityBasedStatusSubsrciber = capturabilityBasedStatusSubsrciber;

      YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("Capturability", capturePointViz.createArtifact());
      YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, YoAppearance.Yellow(), GraphicType.ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("Capturability", desiredCapturePointViz.createArtifact());

      YoArtifactPolygon supportPolygonViz = new YoArtifactPolygon("Combined Polygon", yoSupportPolygon, Color.pink, false);
      yoGraphicsListRegistry.registerArtifact("Capturability", supportPolygonViz);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String name = sidePrefix + "FootSupportPolygon";
         YoFrameConvexPolygon2D yoFootSupportPolygon = new YoFrameConvexPolygon2D(name, "", worldFrame, 4, registry);
         yoFootSupportPolygons.put(robotSide, yoFootSupportPolygon);

         Color color = FootstepListVisualizer.defaultFeetColors.get(robotSide);
         YoArtifactPolygon footSupportPolygonViz = new YoArtifactPolygon(sidePrefix + "Foot Polygon", yoFootSupportPolygon, color, false);
         yoGraphicsListRegistry.registerArtifact("Capturability", footSupportPolygonViz);
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void update(double time)
   {
      FramePoint2D capturePoint = capturabilityBasedStatusSubsrciber.getCapturePoint();
      if (capturePoint != null)
      {
         yoCapturePoint.set(capturePoint);
      }

      FramePoint2D desiredCapturePoint = capturabilityBasedStatusSubsrciber.getDesiredCapturePoint();
      if (desiredCapturePoint != null)
      {
         yoDesiredCapturePoint.set(desiredCapturePoint);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2D footSupportPolygon = capturabilityBasedStatusSubsrciber.getFootSupportPolygon(robotSide);
         if (footSupportPolygon != null)
         {
            yoFootSupportPolygons.get(robotSide).set(footSupportPolygon);
            footSupportPolygons.put(robotSide, footSupportPolygon);
         }
      }

      RobotSide supportLeg;

      if (footSupportPolygons.get(RobotSide.LEFT).isEmpty())
      {
         supportLeg = RobotSide.RIGHT;
         yoSupportPolygon.set(footSupportPolygons.get(supportLeg));
      }
      else if (footSupportPolygons.get(RobotSide.RIGHT).isEmpty())
      {
         supportLeg = RobotSide.LEFT;
         yoSupportPolygon.set(footSupportPolygons.get(supportLeg));
      }
      else
      {
         supportLeg = null;
         supportPolygon.setIncludingFrame(footSupportPolygons.get(RobotSide.LEFT), footSupportPolygons.get(RobotSide.RIGHT));
         yoSupportPolygon.set(supportPolygon);
      }

      yoSupportLeg.set(supportLeg);

      Boolean isInDoubleSupport = capturabilityBasedStatusSubsrciber.isInDoubleSupport();
      if (isInDoubleSupport != null)
      {
         yoDoubleSupport.set(isInDoubleSupport);
         yoSupportLeg.set(null);
      }

      updateCapturePointDistanceToSupportPolygon();

      updateCapturePointError();

      updateTipDetector();
   }

   private void updateTipDetector()
   {
      if (icpError.getDoubleValue() > MAX_CAPTURE_POINT_ERROR_M)
      {
         tippingDetected.set(true);
      }
      else
      {
         tippingDetected.set(false);
      }
   }

   public YoBoolean getTippingDetectedBoolean()
   {
      return tippingDetected;
   }

   public YoFramePoint2D getYoDesiredCapturePoint()
   {
      return yoDesiredCapturePoint;
   }

   public YoFramePoint2D getYoCapturePoint()
   {
      return yoCapturePoint;
   }

   public YoFrameConvexPolygon2D getYoSupportPolygon()
   {
      return yoSupportPolygon;
   }

   public YoFrameConvexPolygon2D getYoFootPolygon(RobotSide robotSide)
   {
      return yoFootSupportPolygons.get(robotSide);
   }

   public YoEnum<RobotSide> getYoSupportLeg()
   {
      return yoSupportLeg;
   }

   public YoBoolean getYoDoubleSupport()
   {
      return yoDoubleSupport;
   }

   public YoDouble getMinIcpDistanceToSupportPolygon()
   {
      return minIcpDistanceToSupportPolygon;
   }

   public YoDouble getIcpError()
   {
      return icpError;
   }

   private Point2D icp = new Point2D();

   private void updateCapturePointDistanceToSupportPolygon()
   {
      icp.set(yoCapturePoint);

      double distanceToClosestEdgeOfSupportPolygon = computeDistanceToClosestEdge(icp, yoSupportPolygon);

      minIcpDistanceToSupportPolygon.set(distanceToClosestEdgeOfSupportPolygon);
   }

   private double computeDistanceToClosestEdge(Point2D pointInsideConvexPolygon, ConvexPolygon2DReadOnly convexPolygon)
   {
      double minDistanceToEdge = Double.POSITIVE_INFINITY;

      double distanceToEdge = 0.0;
      int numberOfVertices = convexPolygon.getNumberOfVertices();

      for (int i = 0; i < numberOfVertices - 1; i++)
      {
         Point2DReadOnly vertex = convexPolygon.getVertex(i);
         Point2DReadOnly vertex2 = convexPolygon.getVertex(i + 1);

         Point2D projectedPoint = projectPointOntoEdge(vertex, vertex2, pointInsideConvexPolygon);

         distanceToEdge = pointInsideConvexPolygon.distance(projectedPoint);

         if (distanceToEdge < minDistanceToEdge)
         {
            minDistanceToEdge = distanceToEdge;
         }
      }
      return minDistanceToEdge;
   }

   private Vector2D edgeVector = new Vector2D();

   private Vector2D constuctEdgeFromTwoVertices(Point2DReadOnly firstVertex, Point2DReadOnly secondVertex)
   {
      edgeVector.set(secondVertex);
      edgeVector.sub(firstVertex);

      return edgeVector;
   }

   private Vector2D firstVertexToPoint = new Vector2D();
   private Point2D projectedPoint = new Point2D();

   private Point2D projectPointOntoEdge(Point2DReadOnly firstVertex, Point2DReadOnly secondVertex, Point2D point)
   {
      Vector2D edgeVector = constuctEdgeFromTwoVertices(firstVertex, secondVertex);

      projectedPoint.set(firstVertex);

      boolean edgeIsNotTrivial = edgeVector.lengthSquared() > 1e-10;
      
      if ( edgeIsNotTrivial )
      {
         firstVertexToPoint.set(point);
         firstVertexToPoint.sub(firstVertex);
         
         double dotProduct = edgeVector.dot(firstVertexToPoint);
         double lengthSquared = edgeVector.lengthSquared();
         double alpha = dotProduct / lengthSquared;

         // Need to keep alpha between 0.0 and 1.0 since if only one edge is seen, the projection can be outside the edge.
         if (alpha < 0.0)
            alpha = 0.0;
         if (alpha > 1.0)
            alpha = 1.0;

         edgeVector.scale(alpha);

         projectedPoint.add(edgeVector);
      }

      return projectedPoint;
   }

   FramePoint2D tempFramePoint2d = new FramePoint2D();

   private void updateCapturePointError()
   {
      tempFramePoint2d.set(yoDesiredCapturePoint.getX(), yoDesiredCapturePoint.getY());

      double error = Math.abs(yoCapturePoint.distance(tempFramePoint2d));

      //      if (error > maxObservedCapturePointError)
      //      {
      //         maxObservedCapturePointError = error;
      ////         System.out.println("TurnValveBehavior: Max Capture Point Error : " + maxObservedCapturePointError);
      //      }
      icpError.set(error);
   }
}
