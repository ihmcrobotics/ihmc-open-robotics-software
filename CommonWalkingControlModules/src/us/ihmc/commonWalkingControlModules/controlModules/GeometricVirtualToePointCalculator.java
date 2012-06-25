package us.ihmc.commonWalkingControlModules.controlModules;

import java.awt.Color;
import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameGeometry2dPlotter;
import us.ihmc.utilities.math.geometry.FrameGeometryTestFrame;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoAppearance;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.time.GlobalTimer;
import com.yobotics.simulationconstructionset.util.graphics.ArtifactList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;


public class GeometricVirtualToePointCalculator implements VirtualToePointCalculator
{
   private static final boolean VISUALIZE = true;
   protected boolean DEBUG_VIZ = false;
   
   private final GlobalTimer globalTimer;
   private final YoVariableRegistry registry = new YoVariableRegistry("VTPCalculator");

   private final SideDependentList<YoFramePoint> virtualToePointsWorld = new SideDependentList<YoFramePoint>();

   private final SideDependentList<ReferenceFrame> ankleZUpFrames;
   private final ReferenceFrame midFeetZUp, world;
   
   private final FrameGeometry2dPlotter plotter;

   public GeometricVirtualToePointCalculator(CommonWalkingReferenceFrames referenceFrames, YoVariableRegistry parentRegistry,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      ankleZUpFrames = referenceFrames.getAnkleZUpReferenceFrames();
      midFeetZUp = referenceFrames.getMidFeetZUpFrame();
      world = ReferenceFrame.getWorldFrame();

      for (RobotSide robotSide : RobotSide.values())
      {         
         YoFramePoint virtualToePointWorld = new YoFramePoint(robotSide + "VTP", "", ReferenceFrame.getWorldFrame(), registry);
         virtualToePointsWorld.put(robotSide, virtualToePointWorld);
      }

      if (VISUALIZE && (dynamicGraphicObjectsListRegistry != null))
      {
         DynamicGraphicObjectsList dynamicGraphicObjectsList = new DynamicGraphicObjectsList("VTP Calculator");
         ArtifactList artifactList = new ArtifactList("VTP Calculator");
         
         for (RobotSide robotSide : RobotSide.values())
         {
            final YoFramePoint virtualToePointWorld = virtualToePointsWorld.get(robotSide);
            DynamicGraphicPosition virtualToePointViz = new DynamicGraphicPosition(robotSide + " VTP", virtualToePointWorld, 0.006, YoAppearance.Orange(),
                  DynamicGraphicPosition.GraphicType.SOLID_BALL);
            dynamicGraphicObjectsList.add(virtualToePointViz);
            artifactList.add(virtualToePointViz.createArtifact());
         }
         
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(dynamicGraphicObjectsList);
         dynamicGraphicObjectsListRegistry.registerArtifactList(artifactList);
      }

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
   }


   /**
    * packVirtualToePoints
    * Sets the VTPs for the feet.
    *
    * @param copDesired FramePoint2d
    */
   public void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints,
         BipedSupportPolygons bipedSupportPolygons, FramePoint2d copDesired, RobotSide upcomingSupportLeg)
   {
      globalTimer.startTimer();

      copDesired.changeFrame(midFeetZUp);

      calculateForDoubleSupport(virtualToePoints, copDesired, bipedSupportPolygons);

      // Visualizer stuff:
      if (VISUALIZE)
      {
         // Virtual toe points:
         for (RobotSide robotSide : RobotSide.values())
         {
            FramePoint vtp3D = virtualToePoints.get(robotSide).toFramePoint();
            vtp3D.changeFrame(world);
            virtualToePointsWorld.get(robotSide).set(vtp3D);
         }
      }

      globalTimer.stopTimer();
   }

   /**
    * calculateForDoubleSupport
    * Sets the VTPs and legStrengths for double support
    *
    * @param coPDesiredInZUp FramePoint2d desired CoP given in any ZUp frame
    */
   private void calculateForDoubleSupport(SideDependentList<FramePoint2d> virtualToePoints,
         FramePoint2d coPDesiredInZUp, BipedSupportPolygons bipedSupportPolygons)
   {
      // 0. Checks:
      coPDesiredInZUp.checkReferenceFrameMatch(midFeetZUp);
      if (!bipedSupportPolygons.getSupportPolygonInMidFeetZUp().isPointInside(coPDesiredInZUp))
      {
         throw new RuntimeException("Desired CoP needs to be inside support polygon.");
      }
      
      if (DEBUG_VIZ)
      {
         plotter.removeAll();
         plotter.addFramePoint2d(coPDesiredInZUp);

         for (RobotSide robotSide : RobotSide.values())
         {
            plotter.addPolygon(bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide));
         }
      }

      // Compute a line that passes through the feasibleCoPDesiredInMidFeetZUp to use for the intersection computation.
      FrameLine2d lineToUseForIntersections = computeLineToUseForIntersectionsUsingConnectingEdgeMorph(bipedSupportPolygons, coPDesiredInZUp);

      for (RobotSide robotSide : RobotSide.values())
      {
         // 4. Find the line segments that have the intersection points of the most constraining lines and the foot polygons as their endpoints:
         final FrameConvexPolygon2d footPolygon = bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide);
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
               throw new RuntimeException("bisector.distance(vtp) > " + epsilonVTP + ". lineToUseForIntersections = " + lineToUseForIntersections
                     + ", vtp = " + vtp + "\nDistance = "
                     + lineToUseForIntersections.distance(vtp));
            }
         }
         else
         {
            if (intersections.length == 1)
               vtp = new FramePoint2d(intersections[0]);
            else if (intersections.length == 2)
            {
               vtp = new FramePoint2d(intersections[0]);
               vtp.add(intersections[1]);
               vtp.scale(0.5);
            }
            else
            {
               throw new RuntimeException("Should never get here.");
            }
         }

         virtualToePoints.put(robotSide, vtp);
      }


      FrameLineSegment2d vtpToVTPLineSegment = new FrameLineSegment2d(virtualToePoints.get(RobotSide.LEFT), virtualToePoints.get(RobotSide.RIGHT));
      if (DEBUG_VIZ)
         plotter.addFrameLineSegment2d(vtpToVTPLineSegment, Color.orange);

      // 6. If the CoP is not on the VTPToVTPLineSegment, adjust the VTPs
      final double epsilonCoPBetweenVTPs = 1.0e-3;
      if (!vtpToVTPLineSegment.isBetweenEndpoints(coPDesiredInZUp, epsilonCoPBetweenVTPs))
      {
         RobotSide sideThatCoPIsIn = determineWhatSideCoPIsIn(bipedSupportPolygons, coPDesiredInZUp);

         if (sideThatCoPIsIn != null)
         {
            virtualToePoints.get(sideThatCoPIsIn).set(coPDesiredInZUp);
         }
         else // cop is not inside either foot
         {
            RobotSide sideThatCoPIsClosestTo = determineWhatSideCoPIsClosestTo(bipedSupportPolygons, coPDesiredInZUp);
            FramePoint2d vtpToChange = virtualToePoints.get(sideThatCoPIsClosestTo);
            vtpToChange.set(coPDesiredInZUp);
            bipedSupportPolygons.getFootPolygonInMidFeetZUp(sideThatCoPIsClosestTo).orthogonalProjection(vtpToChange);
         }
      }

      if (DEBUG_VIZ)
      {
         for (RobotSide robotSide : RobotSide.values())
         {
            plotter.addFramePoint2d(virtualToePoints.get(robotSide));
         }
      }
      
      for (RobotSide robotSide : RobotSide.values())
      {
         virtualToePoints.get(robotSide).changeFrame(ankleZUpFrames.get(robotSide));
      }
   }

   private RobotSide determineWhatSideCoPIsIn(BipedSupportPolygons bipedSupportPolygons, FramePoint2d feasibleCoPDesired)
   {
      RobotSide ret = null;
      for (RobotSide robotSide : RobotSide.values())
      {
         if (bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide).isPointInside(feasibleCoPDesired))
         {
            if (ret == null)
               ret = robotSide;
            else
               throw new RuntimeException("Point is inside both feet!");
         }
      }
      return ret;
   }
   
   private RobotSide determineWhatSideCoPIsClosestTo(BipedSupportPolygons bipedSupportPolygons, FramePoint2d feasibleCoPDesired)
   {
      RobotSide ret = null;
      double closestDistanceToSide = Double.POSITIVE_INFINITY;
      for (RobotSide robotSide : RobotSide.values())
      {
         final double distanceToSide = bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide).distance(feasibleCoPDesired);
         if (distanceToSide < closestDistanceToSide)
         {
            ret = robotSide;
            closestDistanceToSide = distanceToSide;
         }
      }
      return ret;
   }

   private FrameLine2d computeLineToUseForIntersectionsUsingConnectingEdgeMorph(BipedSupportPolygons bipedSupportPolygons,
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
      FramePoint2d[] edge1Points = connectingEdge1.getEndFramepointsCopy();
      FramePoint2d[] edge2Points = connectingEdge2.getEndFramepointsCopy();


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
      FrameVector2d leftFootLeft = linesOfSightLeft[0].getNormalizedFrameVector();
      FrameVector2d leftFootRight = linesOfSightLeft[1].getNormalizedFrameVector();

      FrameVector2d rightFootLeft = linesOfSightRight[0].getNormalizedFrameVector();
      FrameVector2d rightFootRight = linesOfSightRight[1].getNormalizedFrameVector();


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
      double xMin = ref.getLine2dCopy().getNormalizedVectorCopy().x;
      double yMin = ref.getLine2dCopy().getNormalizedVectorCopy().y;
      double xMax = xMin;
      double yMax = yMin;

      FrameLine2d max = ref;

      for (FrameLine2d line : candidates)
      {
         double xCur = line.getLine2dCopy().getNormalizedVectorCopy().x;
         double yCur = line.getLine2dCopy().getNormalizedVectorCopy().y;

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
      for (RobotSide robotSide : RobotSide.values())
      {
         virtualToePointsWorld.get(robotSide).setToNaN();
      }
   }
}
