package us.ihmc.humanoidBehaviors.utilities;

import java.awt.Color;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;

public class CapturePointUpdatable implements Updatable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2d yoDesiredCapturePoint = new YoFramePoint2d("desiredCapturePoint", worldFrame, registry);
   private final YoFramePoint2d yoCapturePoint = new YoFramePoint2d("capturePoint", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoSupportPolygon = new YoFrameConvexPolygon2d("supportPolygon", "", worldFrame, 30, registry);
   private final EnumYoVariable<RobotSide> yoSupportLeg = new EnumYoVariable<>("supportLeg", registry, RobotSide.class, true);
   private final BooleanYoVariable yoDoubleSupport = new BooleanYoVariable("doubleSupport", registry);

   // Computed Stuff
   private final DoubleYoVariable icpError = new DoubleYoVariable("icpError", registry);
   private final DoubleYoVariable minIcpDistanceToSupportPolygon = new DoubleYoVariable("minIcpDistanceToSupportPolygon", registry);
   private final BooleanYoVariable tippingDetected = new BooleanYoVariable("tippingDetected", registry);
   private final double MAX_CAPTURE_POINT_ERROR_M = 0.5 * 0.075; // Reasonable value < 0.01   Max < 0.02

   private final CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber;

   public CapturePointUpdatable(CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
   {
      this.capturabilityBasedStatusSubsrciber = capturabilityBasedStatusSubsrciber;

      YoGraphicPosition capturePointViz = new YoGraphicPosition("Capture Point", yoCapturePoint, 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("Capturability", capturePointViz.createArtifact());
      YoGraphicPosition desiredCapturePointViz = new YoGraphicPosition("Desired Capture Point", yoDesiredCapturePoint, 0.01, YoAppearance.Yellow(),
            GraphicType.ROTATED_CROSS);
      yoGraphicsListRegistry.registerArtifact("Capturability", desiredCapturePointViz.createArtifact());

      YoArtifactPolygon supportPolygonViz = new YoArtifactPolygon("Combined Polygon", yoSupportPolygon, Color.pink, false);
      yoGraphicsListRegistry.registerArtifact("Capturability", supportPolygonViz);

      parentRegistry.addChild(registry);
   }

   @Override
   public void update(double time)
   {
      FramePoint2d capturePoint = capturabilityBasedStatusSubsrciber.getCapturePoint();
      if (capturePoint != null)
      {
         yoCapturePoint.set(capturePoint);
      }

      FramePoint2d desiredCapturePoint = capturabilityBasedStatusSubsrciber.getDesiredCapturePoint();
      if (desiredCapturePoint != null)
      {
         yoDesiredCapturePoint.set(desiredCapturePoint);
      }

      FrameConvexPolygon2d supportPolygon = capturabilityBasedStatusSubsrciber.getSupportPolygon();
      if (supportPolygon != null)
      {
         yoSupportPolygon.setFrameConvexPolygon2d(supportPolygon);
      }

      RobotSide supportLeg = capturabilityBasedStatusSubsrciber.getSupportLeg();
      if (supportLeg != null)
      {
         yoSupportLeg.set(supportLeg);
      }

      Boolean isInDoubleSupport = capturabilityBasedStatusSubsrciber.IsInDoubleSupport();
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

   public BooleanYoVariable getTippingDetectedBoolean()
   {
      return tippingDetected;
   }

   public YoFramePoint2d getYoDesiredCapturePoint()
   {
      return yoDesiredCapturePoint;
   }

   public YoFramePoint2d getYoCapturePoint()
   {
      return yoCapturePoint;
   }

   public YoFrameConvexPolygon2d getYoSupportPolygon()
   {
      return yoSupportPolygon;
   }

   public EnumYoVariable<RobotSide> getYoSupportLeg()
   {
      return yoSupportLeg;
   }

   public BooleanYoVariable getYoDoubleSupport()
   {
      return yoDoubleSupport;
   }

   public DoubleYoVariable getMinIcpDistanceToSupportPolygon()
   {
      return minIcpDistanceToSupportPolygon;
   }

   public DoubleYoVariable getIcpError()
   {
      return icpError;
   }

   private Point2d icp = new Point2d();

   private void updateCapturePointDistanceToSupportPolygon()
   {
      yoCapturePoint.get(icp);

      ConvexPolygon2d supportPolygon = yoSupportPolygon.getConvexPolygon2d();

      double distanceToClosestEdgeOfSupportPolygon = computeDistanceToClosestEdge(icp, supportPolygon);

      minIcpDistanceToSupportPolygon.set(distanceToClosestEdgeOfSupportPolygon);
   }

   private double computeDistanceToClosestEdge(Point2d pointInsideConvexPolygon, ConvexPolygon2d convexPolygon)
   {
      double minDistanceToEdge = Double.POSITIVE_INFINITY;

      double distanceToEdge = 0.0;
      int numberOfVertices = convexPolygon.getNumberOfVertices();

      for (int i = 0; i < numberOfVertices - 1; i++)
      {
         Point2d vertex = convexPolygon.getVertex(i);
         Point2d vertex2 = convexPolygon.getVertex(i + 1);

         Point2d projectedPoint = projectPointOntoEdge(vertex, vertex2, pointInsideConvexPolygon);

         distanceToEdge = pointInsideConvexPolygon.distance(projectedPoint);

         if (distanceToEdge < minDistanceToEdge)
         {
            minDistanceToEdge = distanceToEdge;
         }
      }
      return minDistanceToEdge;
   }

   private Vector2d edgeVector = new Vector2d();

   private Vector2d constuctEdgeFromTwoVertices(Point2d firstVertex, Point2d secondVertex)
   {
      edgeVector.set(secondVertex);
      edgeVector.sub(firstVertex);

      return edgeVector;
   }

   private Vector2d firstVertexToPoint = new Vector2d();
   private Point2d projectedPoint = new Point2d();

   private Point2d projectPointOntoEdge(Point2d firstVertex, Point2d secondVertex, Point2d point)
   {
      Vector2d edgeVector = constuctEdgeFromTwoVertices(firstVertex, secondVertex);

      firstVertexToPoint.set(point);
      firstVertexToPoint.sub(firstVertex);

      projectedPoint.set(firstVertex);

      if (edgeVector.lengthSquared() > 1e-10)
      {
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

   FramePoint2d tempFramePoint2d = new FramePoint2d();

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
