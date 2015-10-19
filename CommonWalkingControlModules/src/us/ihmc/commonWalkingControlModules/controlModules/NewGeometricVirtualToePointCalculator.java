package us.ihmc.commonWalkingControlModules.controlModules;

import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;



public class NewGeometricVirtualToePointCalculator implements VirtualToePointCalculator
{
   private boolean DEBUG_VIZ = false;
   private boolean removeDebugVizEachTime = true;
   
   private final GlobalTimer globalTimer;
   private final YoVariableRegistry registry = new YoVariableRegistry("VTPCalculator");

   private final DoubleYoVariable maximumLegStrengthWhenTransferringAway = new DoubleYoVariable("maximumLegStrengthWhenTransferringAway", registry);

   private SideDependentList<ReferenceFrame> individualFootFramesForReturn;
   private ReferenceFrame commonZUpFrame;
   
   private FrameGeometry2dPlotter plotter;

   public NewGeometricVirtualToePointCalculator(CommonHumanoidReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry, double maximumLegStrengthWhenTransferringAway)
   {
      this(parentRegistry, yoGraphicsListRegistry, maximumLegStrengthWhenTransferringAway);
      this.setFramesToComputeIn(referenceFrames);
   }
   
   public NewGeometricVirtualToePointCalculator(YoVariableRegistry parentRegistry,
         YoGraphicsListRegistry yoGraphicsListRegistry, double maximumLegStrengthWhenTransferringAway)
   {
      globalTimer = new GlobalTimer("VTPCalculatorTimer", registry);

      if ((parentRegistry != null))
      {
         parentRegistry.addChild(registry);
      }

      if (DEBUG_VIZ)
      {
         FrameGeometryTestFrame frame = new FrameGeometryTestFrame(-0.5, 0.5, -0.5, 0.5);
         plotter = frame.getFrameGeometry2dPlotter();
      }
      else
      {
         plotter = null;
      }
      this.maximumLegStrengthWhenTransferringAway.set(maximumLegStrengthWhenTransferringAway);
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
      }
      else
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
      this.commonZUpFrame = ReferenceFrame.getWorldFrame();
      this.individualFootFramesForReturn = new SideDependentList<ReferenceFrame>(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
   }
   
   public void setFramesToComputeIn(ReferenceFrame commonZUpFrame, SideDependentList<ReferenceFrame> individualFootFramesForReturn)
   {
      this.commonZUpFrame = commonZUpFrame;
      this.individualFootFramesForReturn = individualFootFramesForReturn;
   }

   
   public void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints,
         OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d copDesired, RobotSide upcomingSupportSide)
   {
      SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp = bipedSupportPolygons.getFootPolygonsInMidFeetZUp();
      FrameConvexPolygon2d supportPolygonInMidFeetZUp = bipedSupportPolygons.getSupportPolygonInMidFeetZUp();
      FrameLineSegment2d connectingEdge1 = bipedSupportPolygons.getConnectingEdge1();
      FrameLineSegment2d connectingEdge2 = bipedSupportPolygons.getConnectingEdge2();

      packVirtualToePoints(virtualToePoints, copDesired, footPolygonsInMidFeetZUp, supportPolygonInMidFeetZUp, connectingEdge1, connectingEdge2, upcomingSupportSide);
   }


   public void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d copDesired, FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges, RobotSide upcomingSupportSide)
   {
      SideDependentList<FrameConvexPolygon2d> footPolygons = new SideDependentList<FrameConvexPolygon2d>(supportPolygonAndEdges.getOriginalPolygon1(), supportPolygonAndEdges.getOriginalPolygon2());
      packVirtualToePoints(virtualToePoints, copDesired, footPolygons, supportPolygonAndEdges.getFrameConvexPolygon2d(), supportPolygonAndEdges.getConnectingEdge1(), supportPolygonAndEdges.getConnectingEdge2(), upcomingSupportSide);
   }

   public void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints,
         FramePoint2d copDesired, SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp, FrameConvexPolygon2d supportPolygonInMidFeetZUp,
         FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2, RobotSide upcomingSupportSide)
   {
      globalTimer.startTimer();

      copDesired.changeFrame(commonZUpFrame);

      if (copDesired.containsNaN())
      {
         throw new RuntimeException("copDesired.containsNaN()");
      }
      
      calculateForDoubleSupport(virtualToePoints, copDesired, footPolygonsInMidFeetZUp, supportPolygonInMidFeetZUp, connectingEdge1, connectingEdge2);

      globalTimer.stopTimer();
   }

   /**
    * calculateForDoubleSupport
    * Sets the VTPs and legStrengths for double support
    *
    * @param coPDesiredInZUp FramePoint2d desired CoP given in any ZUp frame
    */
   private void calculateForDoubleSupport(SideDependentList<FramePoint2d> virtualToePoints,
         FramePoint2d coPDesiredInZUp, SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp, FrameConvexPolygon2d supportPolygonInMidFeetZUp,
         FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2)
   {
      // 0. Checks:
      coPDesiredInZUp.checkReferenceFrameMatch(commonZUpFrame);

      if (!supportPolygonInMidFeetZUp.isPointInside(coPDesiredInZUp))
      {
         throw new RuntimeException("Desired CoP needs to be inside support polygon.");
      }

     

      // Compute a line that passes through the feasibleCoPDesiredInMidFeetZUp to use for the intersection computation.
//      FrameLine2d lineToUseForIntersections = computeLineToUseForIntersectionsUsingIntersection(connectingEdge1, connectingEdge2, coPDesiredInZUp);
//      FrameLine2d lineToUseForIntersections = computeLineToUseForIntersectionsUsingConnectingEdgeMorph(connectingEdge1, connectingEdge2, coPDesiredInZUp);
      FrameLine2d lineToUseForIntersections = computeLineToUseForIntersectionsUsingProjectionMorph(connectingEdge1, connectingEdge2, coPDesiredInZUp);
      
      SideDependentList<Double> distancesFromFootPolygonToDesired = new SideDependentList<Double>();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint2d closestInsideFoot = new FramePoint2d(coPDesiredInZUp);
         FrameVector2d vector = new FrameVector2d();
         lineToUseForIntersections.getNormalizedFrameVector(vector);
         //         GeometryTools.movePointInsidePolygonAlongLine(closestInsideFoot, footPolygonsInMidFeetZUp.get(robotSide),
//                 lineToUseForIntersections);
         GeometryTools.movePointInsidePolygonAlongVector(closestInsideFoot, vector, footPolygonsInMidFeetZUp.get(robotSide), 0.0);
         double distanceFromFootPolygonToDesired = closestInsideFoot.distance(coPDesiredInZUp);
         distancesFromFootPolygonToDesired.put(robotSide, distanceFromFootPolygonToDesired);
      }
      double distanceFromPolygonToPolygon = distancesFromFootPolygonToDesired.get(RobotSide.LEFT) + distancesFromFootPolygonToDesired.get(RobotSide.RIGHT);
      
      RobotSide sideThatDesiredCoPIsIn = determineWhatSideCoPIsIn(footPolygonsInMidFeetZUp, coPDesiredInZUp);
      RobotSide[] robotSidesInOrder;
      if (sideThatDesiredCoPIsIn == null)
         robotSidesInOrder = RobotSide.values;
      else
         robotSidesInOrder = new RobotSide[] {sideThatDesiredCoPIsIn.getOppositeSide(), sideThatDesiredCoPIsIn};
      
      
      for (RobotSide robotSide : robotSidesInOrder)
      {
         FramePoint2d vtp;

         if (sideThatDesiredCoPIsIn == robotSide)
         {
            FramePoint2d otherVTP = virtualToePoints.get(sideThatDesiredCoPIsIn.getOppositeSide());
            FrameVector2d offset = new FrameVector2d(otherVTP, coPDesiredInZUp);
            double distanceFromOtherVTPToDesiredCoP = offset.length();
            double distanceFromThisVTPToDesiredCoP = distanceFromOtherVTPToDesiredCoP / maximumLegStrengthWhenTransferringAway.getDoubleValue() - distanceFromOtherVTPToDesiredCoP;
            offset.normalize();
            offset.scale(distanceFromThisVTPToDesiredCoP);
            vtp = new FramePoint2d(coPDesiredInZUp);
            vtp.add(offset);
            FrameVector2d vector = new FrameVector2d();
            //            GeometryTools.movePointInsidePolygonAlongLine(vtp, footPolygonsInMidFeetZUp.get(robotSide), lineToUseForIntersections);
            lineToUseForIntersections.getNormalizedFrameVector(vector);
            GeometryTools.movePointInsidePolygonAlongVector(vtp, vector, footPolygonsInMidFeetZUp.get(robotSide), 0.0);
         }
         else
         {
            double distanceFromFootPolygonToDesired = distancesFromFootPolygonToDesired.get(robotSide);
            double lineSegmentParameter = computeLineSegmentParameter(distanceFromFootPolygonToDesired, distanceFromPolygonToPolygon);

            final FrameConvexPolygon2d footPolygon = footPolygonsInMidFeetZUp.get(robotSide);
            FramePoint2d[] intersections = lineToUseForIntersections.intersectionWith(footPolygon);

//            if (DEBUG_VIZ && (intersections != null))
//            {
//               for (int i = 0; i < intersections.length; i++)
//               {
//                  plotter.addFramePoint2d(intersections[i]);
//               }
//            }

            if (intersections == null)
            {
               final double epsilonVTP = 1e-3;
               vtp = footPolygon.getClosestVertexCopy(lineToUseForIntersections);

               // Check to make sure close:
               if (lineToUseForIntersections.distance(vtp) > epsilonVTP)
               {
                  if (DEBUG_VIZ)
                  {
                     plotter.removeAll();
                     plotter.addFramePoint2d(coPDesiredInZUp);

                     for (RobotSide robotSide2 : RobotSide.values)
                     {
                        plotter.addPolygon(footPolygonsInMidFeetZUp.get(robotSide2));
                     }
                     plotter.addFrameLineSegment2d(connectingEdge1, Color.RED);
                     plotter.addFrameLineSegment2d(connectingEdge2, Color.GREEN);
                     plotter.addFrameLine2d(lineToUseForIntersections, Color.ORANGE);
                     plotter.repaint();
                  }
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
                  FrameLineSegment2d lineSegment = new FrameLineSegment2d(intersections);
                  vtp = lineSegment.pointBetweenEndPointsGivenParameter(lineSegmentParameter);
               }
               else
               {
                  throw new RuntimeException("Should never get here.");
               }
            }
         }


         virtualToePoints.put(robotSide, vtp);
      }

//      FrameLineSegment2d vtpToVTPLineSegment = new FrameLineSegment2d(virtualToePoints.get(RobotSide.LEFT), virtualToePoints.get(RobotSide.RIGHT));
//      if (DEBUG_VIZ)
//         plotter.addFrameLineSegment2d(vtpToVTPLineSegment, Color.orange);

//    final double epsilonCoPBetweenVTPs = 1.0e-3;
//    if (!vtpToVTPLineSegment.isBetweenEndpoints(coPDesiredInZUp, epsilonCoPBetweenVTPs))
//    {
//       throw new RuntimeException("Desired CoP is not between VTPs");
//    }

//      if (DEBUG_VIZ)
//      {
//         for (RobotSide robotSide : RobotSide.values)
//         {
//            plotter.addFramePoint2d(virtualToePoints.get(robotSide));
//         }
//      }

      for (RobotSide robotSide : RobotSide.values)
      {
         virtualToePoints.get(robotSide).changeFrame(individualFootFramesForReturn.get(robotSide));
      }
   }

   private double computeLineSegmentParameter(double distanceFromFootPolygonToDesired, double distanceFromPolygonToPolygon)
   {
      double fraction = distanceFromFootPolygonToDesired / distanceFromPolygonToPolygon;
      double growthRate = 5.0;    // looked OK on a Matlab plot
      double ret = 1.0 / (1 + Math.exp(-growthRate * fraction)) - 0.5;    // logistic function; starts steep and then evens out at 0.5

      return ret;
   }

   private RobotSide determineWhatSideCoPIsIn(SideDependentList<FrameConvexPolygon2d> footPolygonsInMidFeetZUp, FramePoint2d coPDesired)
   {
      RobotSide ret = null;
      for (RobotSide robotSide : RobotSide.values)
      {
         if (footPolygonsInMidFeetZUp.get(robotSide).isPointInside(coPDesired))
         {
            if (ret == null)
               ret = robotSide;
            else
               throw new RuntimeException("Point is inside both feet! coPDesired = " + coPDesired);
         }
      }

      return ret;
   }
   
   private FrameLine2d computeLineToUseForIntersectionsUsingIntersection(OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d feasibleCoPDesiredInMidFeetZUp)
   {
      FrameLineSegment2d connectingEdge1 = bipedSupportPolygons.getConnectingEdge1();
      FrameLineSegment2d connectingEdge2 = bipedSupportPolygons.getConnectingEdge2();
      
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

   private FrameLine2d computeLineToUseForIntersectionsUsingProjectionMorph(FrameLineSegment2d connectingEdge1, FrameLineSegment2d connectingEdge2,
         FramePoint2d feasibleCoPDesiredInMidFeetZUp)
   { 
//      if (DEBUG_VIZ)
//      {
//         plotter.addFrameLineSegment2d(connectingEdge1, Color.BLUE);
//         plotter.addFrameLineSegment2d(connectingEdge2, Color.YELLOW);
//      }
    
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
   
   private FrameLine2d computeLineToUseForIntersectionsUsingConnectingEdgeMorph(OldBipedSupportPolygons bipedSupportPolygons,
           FramePoint2d feasibleCoPDesiredInMidFeetZUp)
   {
      FrameLineSegment2d connectingEdge1 = bipedSupportPolygons.getConnectingEdge1();
      FrameLineSegment2d connectingEdge2 = bipedSupportPolygons.getConnectingEdge2();

      if (DEBUG_VIZ)
      {
         plotter.addFrameLineSegment2d(connectingEdge1, Color.BLUE);
         plotter.addFrameLineSegment2d(connectingEdge2, Color.YELLOW);
      }

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
         return new FrameLine2d[] {linesOfSightLeft[1].negateDirectionCopy(), new FrameLine2d(linesOfSightRight[0])};
      }
      else if ((rightFootLeft.cross(leftFootRight) < 0.0) && (rightFootRight.cross(leftFootRight) > 0.0))
      {
         return new FrameLine2d[] {linesOfSightLeft[0].negateDirectionCopy(), new FrameLine2d(linesOfSightRight[1])};
      }

      // Lines of sight are on opposite directions. Invert two and find two inner ones...


      ArrayList<FrameLine2d> candidates = new ArrayList<FrameLine2d>(4);
      candidates.add(linesOfSightLeft[0].negateDirectionCopy());
      candidates.add(linesOfSightLeft[1].negateDirectionCopy());
      candidates.add(new FrameLine2d(linesOfSightRight[0]));
      candidates.add(new FrameLine2d(linesOfSightRight[1]));

      FrameLine2d ref = candidates.get(0);

      FrameLine2d min = ref;
      FrameVector2d vector = new FrameVector2d();
      ref.getNormalizedFrameVector(vector);
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
}
