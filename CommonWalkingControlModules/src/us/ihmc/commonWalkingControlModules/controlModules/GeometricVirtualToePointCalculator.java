package us.ihmc.commonWalkingControlModules.controlModules;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.time.GlobalTimer;


public class GeometricVirtualToePointCalculator implements VirtualToePointCalculator
{
   private boolean DEBUG_VIZ = false;
   private boolean removeDebugVizEachTime = true;

   private final GlobalTimer globalTimer;
   protected final YoVariableRegistry registry = new YoVariableRegistry("VTPCalculator");
   
   private final BooleanYoVariable useLogisticFunction = new BooleanYoVariable("useLogisticFunction", registry);
   private final BooleanYoVariable stickToCoPIfInsideFoot = new BooleanYoVariable("stickToCoPIfInsideFoot", registry);
   
   private final DoubleYoVariable vtpInsideFootMorphPercent = new DoubleYoVariable("vtpInsideFootMorphPercent", registry);
   private final DoubleYoVariable vtpApproachingFootDistance = new DoubleYoVariable("vtpApproachingFootDistance", registry);
   
   private final DoubleYoVariable vtpMorphPercent = new DoubleYoVariable("vtpMorphPercent", registry);

   protected SideDependentList<ReferenceFrame> individualFootFramesForReturn;
   protected ReferenceFrame commonZUpFrame, world;

   private FrameGeometry2dPlotter plotter;

   public GeometricVirtualToePointCalculator(CommonHumanoidReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(parentRegistry, yoGraphicsListRegistry);
      this.setFramesToComputeIn(referenceFrames);
   }

   public GeometricVirtualToePointCalculator(YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      globalTimer = new GlobalTimer("VTPCalculatorTimer", registry);

      if ((parentRegistry != null))
      {
         parentRegistry.addChild(registry);
      }

      setDefaultParameters();
   }

   public void setDefaultParameters()
   {
      setNewR2Parameters();
      
      boolean stickToCoPIfInsideFoot = false;
      boolean useLogisticFunction = false;
      double vtpInsideFootMorphPercent = 0.2; 
      double vtpApproachingFootDistance = 0.10;
      
      setParameters(stickToCoPIfInsideFoot, useLogisticFunction, vtpInsideFootMorphPercent, vtpApproachingFootDistance);
   }
   
   public void setNewR2Parameters()
   {
      boolean stickToCoPIfInsideFoot = true;
      boolean useLogisticFunction = true;
      double vtpInsideFootMorphPercent = 0.0; 
      double vtpApproachingFootDistance = 0.10;
      
      setParameters(stickToCoPIfInsideFoot, useLogisticFunction, vtpInsideFootMorphPercent, vtpApproachingFootDistance);
   }

   public void setParameters(boolean stickToCoPIfInsideFoot, boolean useLogisticFunction, double vtpInsideFootMorphPercent, double vtpApproachingFootDistance)
   {
      this.stickToCoPIfInsideFoot.set(stickToCoPIfInsideFoot);
      this.useLogisticFunction.set(useLogisticFunction);
      this.vtpInsideFootMorphPercent.set(vtpInsideFootMorphPercent);
      this.vtpApproachingFootDistance.set(vtpApproachingFootDistance);
   }

   public synchronized void setupForDebugViz(boolean debugViz, boolean removeDebugVizEachTime)
   {
      this.DEBUG_VIZ = debugViz;
      this.removeDebugVizEachTime = removeDebugVizEachTime;

      if (DEBUG_VIZ)
      {
         FrameGeometryTestFrame frame = new FrameGeometryTestFrame(-0.5, 0.5, -0.5, 0.5);
         plotter = frame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
      } else
      {
         plotter = null;
      }
   }

   public void setFramesToComputeIn(CommonHumanoidReferenceFrames referenceFrames)
   {
      setFramesToComputeIn(referenceFrames.getMidFeetZUpFrame(), referenceFrames.getAnkleZUpReferenceFrames());
   }

   public void setAllFramesToComputeInToWorld()
   {
      this.world = ReferenceFrame.getWorldFrame();
      this.commonZUpFrame = ReferenceFrame.getWorldFrame();
      this.individualFootFramesForReturn = new SideDependentList<ReferenceFrame>(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }

   public void setFramesToComputeIn(ReferenceFrame commonZUpFrame, SideDependentList<ReferenceFrame> individualFootFramesForReturn)
   {
      this.world = ReferenceFrame.getWorldFrame();
      this.commonZUpFrame = commonZUpFrame;
      this.individualFootFramesForReturn = individualFootFramesForReturn;
   }

   public static FramePoint2d projectCoPIntoPolygonAndMoveItOffConnectingEdge(FramePoint2d copDesired,
         FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges, double distanceOffEdge)
   {
      FrameConvexPolygon2d supportPolygon = supportPolygonAndEdges.getFrameConvexPolygon2d();
      FrameLineSegment2d connectingEdge1 = supportPolygonAndEdges.getConnectingEdge1();
      FrameLineSegment2d connectingEdge2 = supportPolygonAndEdges.getConnectingEdge2();
      
      return projectCoPIntoPolygonAndMoveItOffConnectingEdge(copDesired, supportPolygon,
            connectingEdge1, connectingEdge2, distanceOffEdge);
   }

   public static FramePoint2d projectCoPIntoPolygonAndMoveItOffConnectingEdge(FramePoint2d copDesired, FrameConvexPolygon2d supportPolygonInMidFeetZUp,
         FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2, double distanceOffEdge)
   {
      copDesired = supportPolygonInMidFeetZUp.orthogonalProjectionCopy(copDesired);

      FrameLineSegment2d shiftedEdge1 = connectingEdge1.shiftToRightCopy(distanceOffEdge);
      FrameLineSegment2d shiftedEdge2 = connectingEdge2.shiftToRightCopy(distanceOffEdge);

      FramePoint2d ret = copDesired;

      if (shiftedEdge1.isPointOnLeftSideOfLineSegment(copDesired))
      {
         ret = shiftedEdge1.orthogonalProjectionCopy(ret);
      } else if (shiftedEdge2.isPointOnLeftSideOfLineSegment(ret))
      {
         ret = shiftedEdge2.orthogonalProjectionCopy(ret);
      }

      return ret;
   }

   public void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d copDesired,
         RobotSide upcomingSupportSide)
   {
      SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp = bipedSupportPolygons.getFootPolygonsInMidFeetZUp();
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      FrameLineSegment2d connectingEdge1 = bipedSupportPolygons.getConnectingEdge1();
      FrameLineSegment2d connectingEdge2 = bipedSupportPolygons.getConnectingEdge2();

      packVirtualToePoints(virtualToePoints, copDesired, footPolygonsInMidFeetZUp, supportPolygonInMidFeetZUp, connectingEdge1, connectingEdge2,
            upcomingSupportSide);
   }

   public void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d copDesired,
         FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges, RobotSide upcomingSupportSide)
   {
      SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<FrameConvexPolygon2d>(supportPolygonAndEdges.getOriginalPolygon1(),
            supportPolygonAndEdges.getOriginalPolygon2());
      packVirtualToePoints(virtualToePoints, copDesired, footPolygons, supportPolygonAndEdges.getFrameConvexPolygon2d(),
            supportPolygonAndEdges.getConnectingEdge1(), supportPolygonAndEdges.getConnectingEdge2(), upcomingSupportSide);
   }

   public void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d copDesired,
         SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp, FrameConvexPolygon2d supportPolygonInMidFeetZUp, FrameLineSegment2d connectingEdge1,
         FrameLineSegment2d connectingEdge2, RobotSide upcomingSupportSide)
   {
      globalTimer.startTimer();

      if (commonZUpFrame == null)
      {
         throw new RuntimeException("Need to set up frames first with a call to setFramesToComputeIn()");
      }

      copDesired.changeFrame(commonZUpFrame);
      calculateForDoubleSupport(virtualToePoints, copDesired, footPolygonsInMidFeetZUp, supportPolygonInMidFeetZUp, connectingEdge1, connectingEdge2,
            upcomingSupportSide);

      globalTimer.stopTimer();

   }

   protected void calculateForDoubleSupport(SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d coPDesiredInZUp,
         SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp, FrameConvexPolygon2d supportPolygonInMidFeetZUp, FrameLineSegment2d connectingEdge1,
         FrameLineSegment2d connectingEdge2, RobotSide upcomingSupportSide)
   {
      checkReferenceFrameAndPlotInputs(coPDesiredInZUp, footPolygonsInMidFeetZUp, supportPolygonInMidFeetZUp, connectingEdge1, connectingEdge2);

      //      MorphMethod morphMethod = MorphMethod.ORIGINAL;
      MorphMethod morphMethod = MorphMethod.INTERSECTION;
      FrameLine2d lineToUseForIntersections = determineLineToUseForIntersection(coPDesiredInZUp, connectingEdge1, connectingEdge2, morphMethod);

      for (RobotSide robotSide : RobotSide.values)
      {
         // 4. Find the line segments that have the intersection points of the lineToUseForIntersections and the foot polygons as their end points:
         final FrameConvexPolygon2d footPolygon = footPolygonsInMidFeetZUp.get(robotSide);
         FramePoint2d[] intersections = lineToUseForIntersections.intersectionWith(footPolygon);

         if (DEBUG_VIZ && intersections != null)
         {
            for (int i = 0; i < intersections.length; i++)
            {
               plotter.addFramePoint2d(intersections[i]);
            }
         }

         // 5. Calculate the VTPs as the midpoints of these line segments and find the line segment that connects the VTPs:
         FramePoint2d vtp;
         final double epsilonVTP = 1e-3;
         if (intersections == null)
         {
            vtp = footPolygon.getClosestVertexCopy(lineToUseForIntersections);

            // Check to make sure close:
            if (lineToUseForIntersections.distance(vtp) > epsilonVTP)
            {
               throw new RuntimeException("bisector.distance(vtp) > " + epsilonVTP + ". lineToUseForIntersections = " + lineToUseForIntersections + ", vtp = "
                     + vtp + "\nDistance = " + lineToUseForIntersections.distance(vtp));
            }
         } 
         else
         {
            if (intersections.length == 1)
               vtp = new FramePoint2d(intersections[0]);
            else if (intersections.length == 2)
            {
               if (stickToCoPIfInsideFoot.getBooleanValue() && (robotSide == upcomingSupportSide) && footPolygon.isPointInside(coPDesiredInZUp))
               {
                  vtp = new FramePoint2d(coPDesiredInZUp);
               }
               else
               {
               computeVTPMorphPercent(coPDesiredInZUp, upcomingSupportSide, robotSide, intersections);
//               vtp = FramePoint2d.morph(intersections[0], intersections[1], vtpMorphPercent.getDoubleValue());
               vtp = new FramePoint2d();
               vtp.interpolate(intersections[0], intersections[1], vtpMorphPercent.getDoubleValue());
               }
            } else
            {
               throw new RuntimeException("Should never get here.");
            }
         }

         virtualToePoints.put(robotSide, vtp);
      }

      FrameLineSegment2d vtpToVTPLineSegment = new FrameLineSegment2d(virtualToePoints.get(RobotSide.LEFT), virtualToePoints.get(RobotSide.RIGHT));

      // 6. If the CoP is not on the VTPToVTPLineSegment, adjust the VTPs
      final double epsilonCoPBetweenVTPs = 1.0e-3;
      if (!vtpToVTPLineSegment.isBetweenEndpoints(coPDesiredInZUp, epsilonCoPBetweenVTPs))
      {
         RobotSide sideThatCoPIsIn = determineWhatSideCoPIsIn(footPolygonsInMidFeetZUp, coPDesiredInZUp);

         if (sideThatCoPIsIn != null)
         {
            virtualToePoints.get(sideThatCoPIsIn).set(coPDesiredInZUp);
         } else
         // cop is not inside either foot
         {
            RobotSide sideThatCoPIsClosestTo = determineWhatSideCoPIsClosestTo(footPolygonsInMidFeetZUp, coPDesiredInZUp);
            FramePoint2d vtpToChange = virtualToePoints.get(sideThatCoPIsClosestTo);
            vtpToChange.set(coPDesiredInZUp);
            footPolygonsInMidFeetZUp.get(sideThatCoPIsClosestTo).orthogonalProjection(vtpToChange);
         }
      }

      plotOutputs(virtualToePoints);

      for (RobotSide robotSide : RobotSide.values)
      {
         virtualToePoints.get(robotSide).changeFrame(individualFootFramesForReturn.get(robotSide));
      }
   }

   private void computeVTPMorphPercent(FramePoint2d coPDesiredInZUp, RobotSide upcomingSupportSide, RobotSide robotSide, FramePoint2d[] intersections)
   {
      if (useLogisticFunction.getBooleanValue())
      {
         computeVTPMorphPercentLogistic(coPDesiredInZUp, upcomingSupportSide, robotSide, intersections);
      }
      else
      {
         computeVTPMorphPercentLinear(coPDesiredInZUp, upcomingSupportSide, robotSide, intersections);
      }
   }
   
   private void computeVTPMorphPercentLinear(FramePoint2d coPDesiredInZUp, RobotSide upcomingSupportSide, RobotSide robotSide, FramePoint2d[] intersections)
   {
      vtpMorphPercent.set(0.5);
      if (robotSide == upcomingSupportSide)
      {
         double distanceToIntersection = intersections[0].distance(coPDesiredInZUp);
         if (distanceToIntersection < vtpApproachingFootDistance.getDoubleValue())
         {
            double percentNearIntersection = (1.0 - (distanceToIntersection / vtpApproachingFootDistance.getDoubleValue()));
            vtpMorphPercent.set(0.5 - (0.5 - vtpInsideFootMorphPercent.getDoubleValue()) * percentNearIntersection);
            
            if (vtpMorphPercent.getDoubleValue() > 0.5)
               vtpMorphPercent.set(0.5);
            if (vtpMorphPercent.getDoubleValue() < vtpInsideFootMorphPercent.getDoubleValue())
               vtpMorphPercent.set(vtpInsideFootMorphPercent.getDoubleValue());
         }
      }
   }
   
   private void computeVTPMorphPercentLogistic(FramePoint2d coPDesiredInZUp, RobotSide upcomingSupportSide, RobotSide robotSide, FramePoint2d[] intersections)
   {
      vtpMorphPercent.set(0.5);
      if (robotSide == upcomingSupportSide)
      {
         double distanceToIntersection = intersections[0].distance(coPDesiredInZUp);
         if (distanceToIntersection < vtpApproachingFootDistance.getDoubleValue())
         {
            double fraction = distanceToIntersection/vtpApproachingFootDistance.getDoubleValue();
            double growthRate = 5.0;    // looked OK on a Matlab plot
//            double zeroToOne = 1.0 - (2.0 * (1.0 / (1.0 + Math.exp(-growthRate * fraction)) - 0.5));
            double zeroToOne = 2.0 - (2.0 / (1.0 + Math.exp(-growthRate * fraction)));
             
            vtpMorphPercent.set(0.5 - (0.5 - vtpInsideFootMorphPercent.getDoubleValue()) * zeroToOne);

            if (vtpMorphPercent.getDoubleValue() > 0.5)
               vtpMorphPercent.set(0.5);
            if (vtpMorphPercent.getDoubleValue() < vtpInsideFootMorphPercent.getDoubleValue())
               vtpMorphPercent.set(vtpInsideFootMorphPercent.getDoubleValue());
         }
      }
   }

   protected void plotOutputs(SideDependentList<FramePoint2d> virtualToePoints)
   {
      FrameLineSegment2d vtpToVTPLineSegment;
      if (DEBUG_VIZ)
      {
         vtpToVTPLineSegment = new FrameLineSegment2d(virtualToePoints.get(RobotSide.LEFT), virtualToePoints.get(RobotSide.RIGHT));
         plotter.addFrameLineSegment2d(vtpToVTPLineSegment, Color.orange);

         for (RobotSide robotSide : RobotSide.values)
         {
            plotter.addFramePoint2d(virtualToePoints.get(robotSide), Color.CYAN);
         }

         plotter.repaint();
      }
   }

   protected FrameLine2d determineLineToUseForIntersection(FramePoint2d coPDesiredInZUp, FrameLineSegment2d connectingEdge1,
         FrameLineSegment2d connectingEdge2, MorphMethod morphMethod)
   {
      // Compute a line that passes through the coPDesiredInZUp to use for the intersection computation.

      FrameLine2d lineToUseForIntersections;
      switch (morphMethod)
      {
      case ORIGINAL:
      {
         lineToUseForIntersections = computeLineToUseForIntersectionsUsingConnectingEdgeMorph(connectingEdge1, connectingEdge2, coPDesiredInZUp);
         break;
      }
      case INTERSECTION:
      {
         lineToUseForIntersections = computeLineToUseForIntersectionsUsingProjectionMorph(connectingEdge1, connectingEdge2, coPDesiredInZUp);
         break;
      }
      case PROJECTION:
      {
         lineToUseForIntersections = computeLineToUseForIntersectionsUsingIntersection(connectingEdge1, connectingEdge2, coPDesiredInZUp);
         break;
      }
      default:
      {
         throw new RuntimeException("Shouldn't get here");
      }
      }
      return lineToUseForIntersections;
   }

   protected void checkReferenceFrameAndPlotInputs(FramePoint2d coPDesiredInZUp, SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp,
         FrameConvexPolygon2d supportPolygonInMidFeetZUp, FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2)
   {
      coPDesiredInZUp.checkReferenceFrameMatch(commonZUpFrame);
      if (!supportPolygonInMidFeetZUp.isPointInside(coPDesiredInZUp))
      {
         throw new RuntimeException("Desired CoP needs to be inside support polygon.");
      }

      if (DEBUG_VIZ)
      {
         if (removeDebugVizEachTime)
            plotter.removeAllObjectsToDraw();
         plotter.addFramePoint2d(coPDesiredInZUp, Color.MAGENTA);

         for (RobotSide robotSide : RobotSide.values)
         {
            plotter.addPolygon(footPolygonsInMidFeetZUp.get(robotSide));
         }

         plotter.addFrameLineSegment2d(connectingEdge1, Color.BLUE);
         plotter.addFrameLineSegment2d(connectingEdge2, Color.YELLOW);
      }
   }

   private RobotSide determineWhatSideCoPIsIn(SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp, FramePoint2d feasibleCoPDesired)
   {
      RobotSide ret = null;
      for (RobotSide robotSide : RobotSide.values)
      {
         if (footPolygonsInMidFeetZUp.get(robotSide).isPointInside(feasibleCoPDesired))
         {
            if (ret == null)
               ret = robotSide;
            else
               throw new RuntimeException("Point is inside both feet!");
         }
      }
      return ret;
   }

   private RobotSide determineWhatSideCoPIsClosestTo(SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp, FramePoint2d feasibleCoPDesired)
   {
      RobotSide ret = null;
      double closestDistanceToSide = Double.POSITIVE_INFINITY;
      for (RobotSide robotSide : RobotSide.values)
      {
         final double distanceToSide = footPolygonsInMidFeetZUp.get(robotSide).distance(feasibleCoPDesired);
         if (distanceToSide < closestDistanceToSide)
         {
            ret = robotSide;
            closestDistanceToSide = distanceToSide;
         }
      }
      return ret;
   }

   protected FrameLine2d computeLineToUseForIntersectionsUsingConnectingEdgeMorph(FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2,
         FramePoint2d feasibleCoPDesiredInMidFeetZUp)
   {
      connectingEdge1.checkReferenceFrameMatch(feasibleCoPDesiredInMidFeetZUp);
      connectingEdge2.checkReferenceFrameMatch(feasibleCoPDesiredInMidFeetZUp);

      // Solve for intersecting line.
      FramePoint2d[] edge1Points = connectingEdge1.getEndFramePointsCopy();
      FramePoint2d[] edge2Points = connectingEdge2.getEndFramePointsCopy();

      FramePoint2d P1 = edge1Points[0];
      FramePoint2d P2 = edge2Points[1];

      if (DEBUG_VIZ)
      {
         plotter.addFramePoint2d(P1, Color.orange);
         plotter.addFramePoint2d(P2, Color.orange);
      }

      FrameVector2d V1 = new FrameVector2d(edge1Points[1]);
      V1.sub(P1);

      FrameVector2d V2 = new FrameVector2d(edge2Points[0]);
      V2.sub(P2);

      FrameVector2d Q = new FrameVector2d(P2);
      Q.sub(P1);

      FrameVector2d R = new FrameVector2d(V2);
      R.sub(V1);

      FrameVector2d S = new FrameVector2d(feasibleCoPDesiredInMidFeetZUp);
      S.sub(P1);

      double A = Q.getX() * R.getY() - Q.getY() * R.getX();
      double B = R.getX() * S.getY() - R.getY() * S.getX() - V1.getX() * Q.getY() + Q.getX() * V1.getY();
      double C = V1.getX() * S.getY() - V1.getY() * S.getX();

      double alpha;
      if (Math.abs(A) < 1e-7)
         alpha = -C / B;
      else
      {
         double B2Minus4AC = B * B - 4.0 * A * C;

         if (B2Minus4AC < 0.0)
         {
            if (B2Minus4AC < -1e-7)
               throw new RuntimeException("Should never get here!");
            B2Minus4AC = 0.0;
         }

         alpha = (-B + Math.sqrt(B2Minus4AC)) / (2.0 * A);
      }

      FrameVector2d V = new FrameVector2d(R);
      V.scale(alpha);
      V.add(V1);

      FrameLine2d ret = new FrameLine2d(feasibleCoPDesiredInMidFeetZUp, V);

      if (DEBUG_VIZ)
      {
         plotter.addFrameLine2d(ret, Color.RED);
      }

      return ret;
   }

   protected FrameLine2d computeLineToUseForIntersectionsUsingProjectionMorph(FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2,
         FramePoint2d feasibleCoPDesiredInMidFeetZUp)
   {
      connectingEdge1.checkReferenceFrameMatch(feasibleCoPDesiredInMidFeetZUp);
      connectingEdge2.checkReferenceFrameMatch(feasibleCoPDesiredInMidFeetZUp);

      Line2d connectingLine1 = new Line2d(connectingEdge1.getLineSegment2d().getEndpoints()[0], connectingEdge1.getLineSegment2d().getEndpoints()[1]);
      Line2d connectingLine2 = new Line2d(connectingEdge2.getLineSegment2d().getEndpoints()[1], connectingEdge2.getLineSegment2d().getEndpoints()[0]);
      Point2d copPoint2d = feasibleCoPDesiredInMidFeetZUp.getPoint();

      double distance1 = connectingLine1.distance(copPoint2d);
      double distance2 = connectingLine2.distance(copPoint2d);
      double distanceSum = distance1 + distance2;

      double alpha = distance2 / distanceSum;

      Vector2d vector = new Vector2d();
      connectingLine1.getNormalizedVector(vector);
      Vector2d vector2 = new Vector2d();
      connectingLine2.getNormalizedVector(vector2);

      vector.scale(alpha);
      vector2.scale(1.0 - alpha);
      vector.add(vector2);
      return new FrameLine2d(feasibleCoPDesiredInMidFeetZUp.getReferenceFrame(), copPoint2d, vector);
   }

   protected FrameLine2d computeLineToUseForIntersectionsUsingIntersection(FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2,
         FramePoint2d feasibleCoPDesiredInMidFeetZUp)
   {
      FramePoint2d[] edge1Points = connectingEdge1.getEndFramePointsCopy();
      FramePoint2d[] edge2Points = connectingEdge2.getEndFramePointsCopy();

      FrameVector2d V1 = new FrameVector2d(edge1Points[1]);
      V1.sub(edge1Points[0]);
      V1.normalize();

      FrameVector2d V2 = new FrameVector2d(edge2Points[0]);
      V2.sub(edge2Points[1]);
      V2.normalize();

      FrameLineSegment2d edge = new FrameLineSegment2d(edge1Points[0], edge2Points[1]);
      double parameter = edge.percentageAlongLineSegment(feasibleCoPDesiredInMidFeetZUp);

      V1.scale(1.0 - parameter);
      V2.scale(parameter);

      FrameVector2d vector = new FrameVector2d(V1);
      vector.add(V2);
      return new FrameLine2d(feasibleCoPDesiredInMidFeetZUp, vector);
   }

   /**
    * mostConstrainingLines
    * Finds the 'medians' (sort of) of the lines passed in.
    *
    * @param linesOfSightRight FrameLine2d[]
    * @param linesOfSightLeft FrameLine2d[]
    * @return FrameLine2d[]
    */
   public static final FrameLine2d[] mostConstrainingLines(FrameLine2d[] linesOfSightRight, FrameLine2d[] linesOfSightLeft)
   {
      FrameVector2d leftFootLeft = new FrameVector2d();
      linesOfSightLeft[0].getNormalizedFrameVector(leftFootLeft);
      FrameVector2d leftFootRight = new FrameVector2d();
      linesOfSightLeft[1].getNormalizedFrameVector(leftFootRight);

      FrameVector2d rightFootLeft = new FrameVector2d();
      linesOfSightRight[0].getNormalizedFrameVector(rightFootLeft);
      FrameVector2d rightFootRight = new FrameVector2d();
      linesOfSightRight[1].getNormalizedFrameVector(rightFootRight);

      if ((rightFootLeft.cross(leftFootLeft) < 0.0) && (rightFootRight.cross(leftFootLeft) > 0.0))
      {
         return new FrameLine2d[] { linesOfSightLeft[1].negateDirectionCopy(), new FrameLine2d(linesOfSightRight[0]) };
      } else if ((rightFootLeft.cross(leftFootRight) < 0.0) && (rightFootRight.cross(leftFootRight) > 0.0))
      {
         return new FrameLine2d[] { linesOfSightLeft[0].negateDirectionCopy(), new FrameLine2d(linesOfSightRight[1]) };
      }

      // Lines of sight are on opposite directions. Invert two and find two inner ones...

      ArrayList<FrameLine2d> candidates = new ArrayList<FrameLine2d>(4);
      candidates.add(linesOfSightLeft[0].negateDirectionCopy());
      candidates.add(linesOfSightLeft[1].negateDirectionCopy());
      candidates.add(new FrameLine2d(linesOfSightRight[0]));
      candidates.add(new FrameLine2d(linesOfSightRight[1]));

      FrameLine2d ref = candidates.get(0);
      FrameVector2d vector = new FrameVector2d();
      ref.getNormalizedFrameVector(vector);
      FrameLine2d min = ref;
      double xMin = vector.getX();
      double yMin = vector.getY();
      double xMax = xMin;
      double yMax = yMin;

      FrameLine2d max = ref;

      for (FrameLine2d line : candidates)
      {
         line.getNormalizedFrameVector(vector);
         double xCur = vector.getX();
         double yCur = vector.getY();

         double crossProductWithMin = xMin * yCur - yMin * xCur;

         if (crossProductWithMin < 0)
         {
            min = line;
            xMin = xCur;
            yMin = yCur;
         }

         double crossProductWithMax = xMax * yCur - yMax * xCur;
         if (crossProductWithMax > 0)
         {
            max = line;
            xMax = xCur;
            yMax = yCur;
         }
      }

      candidates.remove(min);
      candidates.remove(max);

      FrameLine2d[] mostConstrainingLines = new FrameLine2d[2];
      mostConstrainingLines[0] = candidates.get(0);
      mostConstrainingLines[1] = candidates.get(1);

      return mostConstrainingLines;
   }

   public void hideVisualizationGraphics()
   {
   }

   protected enum MorphMethod
   {
      ORIGINAL, PROJECTION, INTERSECTION
   }
}
