package us.ihmc.humanoidBehaviors.utilities;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.communication.subscribers.CapturabilityBasedStatusSubscriber;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class CapturePointUpdatable implements Updatable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint2d yoDesiredCapturePoint = new YoFramePoint2d("desiredCapturePoint", worldFrame, registry);
   private final YoFramePoint2d yoCapturePoint = new YoFramePoint2d("capturePoint", worldFrame, registry);
   private final YoFrameConvexPolygon2d yoSupportPolygon = new YoFrameConvexPolygon2d("supportPolygon", "", worldFrame, 30, registry);
   private final SideDependentList<YoFrameConvexPolygon2d> yoFootSupportPolygons = new SideDependentList<>();
   private final EnumYoVariable<RobotSide> yoSupportLeg = new EnumYoVariable<>("supportLeg", registry, RobotSide.class, true);
   private final BooleanYoVariable yoDoubleSupport = new BooleanYoVariable("doubleSupport", registry);

   // Computed Stuff
   private final DoubleYoVariable icpError = new DoubleYoVariable("icpError", registry);
   private final DoubleYoVariable minIcpDistanceToSupportPolygon = new DoubleYoVariable("minIcpDistanceToSupportPolygon", registry);
   private final BooleanYoVariable tippingDetected = new BooleanYoVariable("tippingDetected", registry);
   private final double MAX_CAPTURE_POINT_ERROR_M = 0.5 * 0.075; // Reasonable value < 0.01   Max < 0.02

   private final FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
   private final SideDependentList<FrameConvexPolygon2d> footSupportPolygons = new SideDependentList<>(new FrameConvexPolygon2d(), new FrameConvexPolygon2d());

   private final CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber;

   public CapturePointUpdatable(CapturabilityBasedStatusSubscriber capturabilityBasedStatusSubsrciber, YoGraphicsListRegistry yoGraphicsListRegistry,
         YoVariableRegistry parentRegistry)
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
         YoFrameConvexPolygon2d yoFootSupportPolygon = new YoFrameConvexPolygon2d(name, "", worldFrame, 4, registry);
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

      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d footSupportPolygon = capturabilityBasedStatusSubsrciber.getFootSupportPolygon(robotSide);
         if (footSupportPolygon != null)
         {
            yoFootSupportPolygons.get(robotSide).setFrameConvexPolygon2d(footSupportPolygon);
            footSupportPolygons.put(robotSide, footSupportPolygon);
         }
      }

      RobotSide supportLeg;

      if (footSupportPolygons.get(RobotSide.LEFT).isEmpty())
      {
         supportLeg = RobotSide.RIGHT;
         yoSupportPolygon.setFrameConvexPolygon2d(footSupportPolygons.get(supportLeg));
      }
      else if (footSupportPolygons.get(RobotSide.RIGHT).isEmpty())
      {
         supportLeg = RobotSide.LEFT;
         yoSupportPolygon.setFrameConvexPolygon2d(footSupportPolygons.get(supportLeg));
      }
      else
      {
         supportLeg = null;
         supportPolygon.setIncludingFrameAndUpdate(footSupportPolygons.get(RobotSide.LEFT), footSupportPolygons.get(RobotSide.RIGHT));
         yoSupportPolygon.setFrameConvexPolygon2d(supportPolygon);
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

   public YoFrameConvexPolygon2d getYoFootPolygon(RobotSide robotSide)
   {
      return yoFootSupportPolygons.get(robotSide);
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

   private Point2D icp = new Point2D();

   private void updateCapturePointDistanceToSupportPolygon()
   {
      yoCapturePoint.get(icp);

      ConvexPolygon2d supportPolygon = yoSupportPolygon.getConvexPolygon2d();

      double distanceToClosestEdgeOfSupportPolygon = computeDistanceToClosestEdge(icp, supportPolygon);

      minIcpDistanceToSupportPolygon.set(distanceToClosestEdgeOfSupportPolygon);
   }

   private double computeDistanceToClosestEdge(Point2D pointInsideConvexPolygon, ConvexPolygon2d convexPolygon)
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
