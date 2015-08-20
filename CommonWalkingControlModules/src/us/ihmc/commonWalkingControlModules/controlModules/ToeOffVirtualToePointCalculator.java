package us.ihmc.commonWalkingControlModules.controlModules;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.OldBipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SimpleDesiredCapturePointCalculator;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameConvexPolygon2dAndConnectingEdges;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.yoUtilities.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;


public class ToeOffVirtualToePointCalculator implements VirtualToePointCalculator
{

   private final YoVariableRegistry registry = new YoVariableRegistry("ToeOffVirtualToePointCalculator");
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final CouplingRegistry couplingRegistry;
   private final SideDependentList<FramePoint> footPoints = new SideDependentList<FramePoint>();
   private final SideDependentList<FramePoint[]> toePoints = new SideDependentList<FramePoint[]>();
   private final SideDependentList<FramePoint[]> alternativeToePoints = new SideDependentList<FramePoint[]>();
   private final SideDependentList<YoFramePoint> virtualToePointsWorld = new SideDependentList<YoFramePoint>();
   private final GeometricVirtualToePointCalculator geometricVirtualToePointCalculator;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoFrameConvexPolygon2d vtpConvexPolygon = new YoFrameConvexPolygon2d("vtpConvexPolygon", "", worldFrame, 6, registry);

   private final DoubleYoVariable minICPToEdgeY = new DoubleYoVariable("minICPToEdgeY", registry);

   private final DoubleYoVariable pullBackSupportLine = new DoubleYoVariable("pullBackSupportLine", registry);

   private final ReferenceFrame midFeetZUpFrame;

   private final double footLength;

   public ToeOffVirtualToePointCalculator(CommonHumanoidReferenceFrames referenceFrames, CouplingRegistry couplingRegistry, double footForward, double footBack,
         double footWidth, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      SimpleDesiredCapturePointCalculator.USEUPCOMINGSWINGTOEASSWEETSPOT = true; // TODO: AAAAAAAAAAAAAAAAAAAARRRRRRRRRRRRRGHHHHHHHHHHHHHHHH

      this.referenceFrames = referenceFrames;
      this.couplingRegistry = couplingRegistry;
      midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

      for (RobotSide robotSide : RobotSide.values)
      {

         footPoints.put(robotSide, new FramePoint(referenceFrames.getAnkleZUpFrame(robotSide)));

         FramePoint insideToe = new FramePoint(referenceFrames.getFootFrame(robotSide), footForward, (robotSide == RobotSide.LEFT ? -1.0 : 1.0) * footWidth
               / 2.0, 0.0);
         FramePoint outsideToe = new FramePoint(referenceFrames.getFootFrame(robotSide), footForward, (robotSide == RobotSide.LEFT ? 1.0 : -1.0) * footWidth
               / 2.0, 0.0);

         FramePoint[] toePointsForFeet = { insideToe, outsideToe };
         toePoints.put(robotSide, toePointsForFeet);

         FramePoint alternativeInsideToe = new FramePoint(referenceFrames.getFootFrame(robotSide), footForward, 0.0, 0.0);
         FramePoint alternativeOutsideToe = new FramePoint(referenceFrames.getFootFrame(robotSide), -footBack, 0.0, 0.0);

         FramePoint[] alternativeToePointsForFeet = { alternativeInsideToe, alternativeOutsideToe };
         alternativeToePoints.put(robotSide, alternativeToePointsForFeet);

         YoFramePoint virtualToePointWorld = new YoFramePoint(robotSide + "VTP", "", ReferenceFrame.getWorldFrame(), registry);
         virtualToePointsWorld.put(robotSide, virtualToePointWorld);

      }

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicsList yoGraphicsList = new YoGraphicsList("VTP Calculator");
         ArtifactList artifactList = new ArtifactList("VTP Calculator");

         YoArtifactPolygon vtpConvexPolygonArtifact = new YoArtifactPolygon("vtpConvexPolygon", vtpConvexPolygon, Color.cyan, false);
         artifactList.add(vtpConvexPolygonArtifact);

         for (RobotSide robotSide : RobotSide.values)
         {
            final YoFramePoint virtualToePointWorld = virtualToePointsWorld.get(robotSide);
            YoGraphicPosition virtualToePointViz = new YoGraphicPosition(robotSide.getCamelCaseNameForStartOfExpression() + "VTP",
                  virtualToePointWorld, 0.006, YoAppearance.Orange(), YoGraphicPosition.GraphicType.SOLID_BALL);
            yoGraphicsList.add(virtualToePointViz);
            artifactList.add(virtualToePointViz.createArtifact());
         }

         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
         yoGraphicsListRegistry.registerArtifactList(artifactList);

      }
      geometricVirtualToePointCalculator = new GeometricVirtualToePointCalculator(referenceFrames, parentRegistry, yoGraphicsListRegistry);
      footLength = (footBack + footForward) * 0.7;

      minICPToEdgeY.set(0.05);

      parentRegistry.addChild(registry);
   }

   public void packVirtualToePoints(SideDependentList<FramePoint2d> virtualToePoints, OldBipedSupportPolygons bipedSupportPolygons, FramePoint2d copDesired,
         RobotSide upcomingSupportLeg)
   {
      // Find trailing leg
      RobotSide upcomingSwingLeg = upcomingSupportLeg.getOppositeSide();

      // Create new support polygon
      FrameConvexPolygon2d footPolygonInMidFeetZUp = bipedSupportPolygons.getFootPolygonInMidFeetZUp(upcomingSupportLeg);
      FramePoint[] toePointsForFeet = toePoints.get(upcomingSwingLeg);

      // Choose toe points to use
      FramePoint upcomingSupportFootPosition = new FramePoint(footPoints.get(upcomingSupportLeg));
      upcomingSupportFootPosition.changeFrame(toePointsForFeet[0].getReferenceFrame());

      // If upcoming support foot is not in front of upcoming swing foot
      boolean useAlternateToePoints = !(toePointsForFeet[0].getX() < upcomingSupportFootPosition.getX() || toePointsForFeet[1].getX() < upcomingSupportFootPosition
            .getX());
      if (useAlternateToePoints)
      {
         geometricVirtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, copDesired, upcomingSupportLeg);
         return;
      }

      FramePoint insideToe = new FramePoint(toePointsForFeet[0]);
      insideToe.changeFrame(midFeetZUpFrame);
      FramePoint outsideToe = new FramePoint(toePointsForFeet[1]);
      outsideToe.changeFrame(midFeetZUpFrame);
      FrameVector adjustment = new FrameVector(referenceFrames.getFootFrame(upcomingSwingLeg), -1.0, 0.0, 0.0);
      adjustment.changeFrame(midFeetZUpFrame);
      adjustment.scale(pullBackSupportLine.getDoubleValue());
      insideToe.add(adjustment);
      outsideToe.add(adjustment);

      FramePoint2d insideToe2d = insideToe.toFramePoint2d();
      FramePoint2d outsideToe2d = outsideToe.toFramePoint2d();

      // Check if iCP is in support polygon
      FramePoint2d icp = couplingRegistry.getCapturePointInFrame(midFeetZUpFrame).toFramePoint2d();

      FramePoint2d sweetSpotUpcomingSwingLeg = bipedSupportPolygons.getSweetSpotCopy(upcomingSwingLeg);
      FramePoint2d sweetSpotUpcomingSupportLeg = bipedSupportPolygons.getSweetSpotCopy(upcomingSupportLeg);
      sweetSpotUpcomingSupportLeg.changeFrame(midFeetZUpFrame);
      sweetSpotUpcomingSwingLeg.changeFrame(midFeetZUpFrame);

      FrameLineSegment2d sweetSpotToSweetSpot = new FrameLineSegment2d(sweetSpotUpcomingSwingLeg, sweetSpotUpcomingSupportLeg);

      sweetSpotToSweetSpot.orthogonalProjection(icp);

      double position = MathTools.clipToMinMax(sweetSpotToSweetSpot.percentageAlongLineSegment(icp), 0.0, 1.0);

      if (position < 0.5)
      {
         FrameVector direction = new FrameVector(referenceFrames.getFootFrame(upcomingSwingLeg), -1.0, 0.0, 0.0);
         direction.changeFrame(midFeetZUpFrame);
         direction.scale((1.0 - position * 2.0) * (footLength - pullBackSupportLine.getDoubleValue()));
         insideToe2d.add(direction.toFrameVector2d());
         outsideToe2d.add(direction.toFrameVector2d());
      }

      //         FramePoint2d icpInside = instantaneousCapturePoint;
      //         FramePoint2d icpOutside = new FramePoint2d(instantaneousCapturePoint);
      //
      //         icpInside.setY(icpInside.getY() + (upcomingSwingLeg == RobotSide.LEFT ? -1.0 : 1.0) * minICPToEdgeY.getDoubleValue());
      //         icpOutside.setY(icpOutside.getY() + (upcomingSwingLeg == RobotSide.LEFT ? 1.0 : -1.0) * minICPToEdgeY.getDoubleValue());
      //
      //         icpInside.changeFrame(midFeetZUpFrame);
      //         icpOutside.changeFrame(midFeetZUpFrame);
      //
      //         FrameLine2d insideToeLine = new FrameLine2d(insideToe2d, icpInside);
      //         FrameLine2d outsideToeLine = new FrameLine2d(outsideToe2d, icpOutside);
      //
      //         boolean insideToeLineIntersect = footPolygonInMidFeetZUp.intersectionWith(insideToeLine) != null;
      //         boolean insideToeIsOk = true;
      //         boolean outsideToeLineIntersect = footPolygonInMidFeetZUp.intersectionWith(outsideToeLine) != null;
      //
      //         FrameVector direction = new FrameVector(referenceFrames.getFootFrame(upcomingSwingLeg), -1.0, 0.0, 0.0);
      //         direction.changeFrame(midFeetZUpFrame);
      //
      //         if (!insideToeLineIntersect)
      //         {
      //            FramePoint2d closestVertex = footPolygonInMidFeetZUp.getClosestVertexCopy(insideToeLine);
      //            FrameVector2d vectorFromPolygon = new FrameVector2d(closestVertex, insideToeLine.orthogonalProjectionCopy(closestVertex));
      //
      //            vectorFromPolygon.changeFrame(referenceFrames.getFootFrame(upcomingSupportLeg));
      //
      //            if (vectorFromPolygon.getX() * (upcomingSupportLeg == RobotSide.LEFT ? 1.0 : -1.0) > 0.0)
      //            {
      //               FrameLine2d insideOfFoot = new FrameLine2d(insideToe2d, direction.toFrameVector2d());
      //               FrameLine2d crossLine = new FrameLine2d(closestVertex, icpInside);
      //               FramePoint2d newInsideToePoint = crossLine.intersectionWith(insideOfFoot);
      //
      //               FrameVector2d changeVector = new FrameVector2d(insideToe2d, newInsideToePoint);
      //               double length = changeVector.length();
      //               if (length > footLength)
      //               {
      //                  changeVector.set(direction.toFrameVector2d());
      //                  changeVector.scale(footLength);
      //               }
      //
      //               insideToe2d.add(changeVector);
      //               outsideToe2d.add(changeVector);
      //
      //               insideToeIsOk = false;
      //            }
      //         }
      //
      //         if (insideToeIsOk && !outsideToeLineIntersect)
      //         {
      //            FramePoint2d closestVertex = footPolygonInMidFeetZUp.getClosestVertexCopy(outsideToeLine);
      //            FrameVector2d vectorFromPolygon = new FrameVector2d(closestVertex, outsideToeLine.orthogonalProjectionCopy(closestVertex));
      //
      //            vectorFromPolygon.changeFrame(referenceFrames.getFootFrame(upcomingSupportLeg));
      //
      //            if (vectorFromPolygon.getX() * (upcomingSupportLeg == RobotSide.LEFT ? -1.0 : 1.0) > 0.0)
      //            {
      //               FrameLine2d outsideOfFoot = new FrameLine2d(outsideToe2d, direction.toFrameVector2d());
      //               FrameLine2d crossLine = new FrameLine2d(closestVertex, icpOutside);
      //               FramePoint2d newOutsideToePoint = crossLine.intersectionWith(outsideOfFoot);
      //
      //               FrameVector2d changeVector = new FrameVector2d(insideToe2d, newOutsideToePoint);
      //               double length = changeVector.length();
      //               if (length > footLength)
      //               {
      //                  changeVector.set(direction.toFrameVector2d());
      //                  changeVector.scale(footLength);
      //               }
      //               outsideToe2d.add(changeVector);
      //               insideToe2d.add(changeVector);
      //
      //            }
      //         }

      FrameLineSegment2d toeLine = new FrameLineSegment2d(insideToe2d, outsideToe2d);

      FrameConvexPolygon2d adjustedSupportPolygon = new FrameConvexPolygon2d();
      adjustedSupportPolygon.clear(footPolygonInMidFeetZUp.getReferenceFrame());
      adjustedSupportPolygon.addVertices(footPolygonInMidFeetZUp);
      adjustedSupportPolygon.addVertex(insideToe2d);
      adjustedSupportPolygon.addVertex(outsideToe2d);
      adjustedSupportPolygon.update();
      FrameConvexPolygon2d temp = new FrameConvexPolygon2d(adjustedSupportPolygon);
      temp.changeFrame(worldFrame);
      vtpConvexPolygon.setFrameConvexPolygon2d(temp);

      FrameLine2d connectingEdgeA = null;
      FrameLine2d connectingEdgeB = null;

      // TODO: Improve algorithm to find connecting edges to handle all corner cases.
      double epsilon = 1e-6;
      int numberOfPoints = adjustedSupportPolygon.getNumberOfVertices();
      boolean prevEqualsInsideToe = insideToe2d.epsilonEquals(adjustedSupportPolygon.getFrameVertex(numberOfPoints - 1), epsilon);
      boolean prevEqualsOutsideToe = outsideToe2d.epsilonEquals(adjustedSupportPolygon.getFrameVertex(numberOfPoints - 1), epsilon);
      boolean equalsInsideToe = insideToe2d.epsilonEquals(adjustedSupportPolygon.getFrameVertex(0), epsilon);
      boolean equalsOutsideToe = outsideToe2d.epsilonEquals(adjustedSupportPolygon.getFrameVertex(0), epsilon);

      boolean placeVTPOnOutsideToe = false, placeVTPOnInsideToe = false;
      for (int i = 0; i < numberOfPoints; i++)
      {
         boolean nextEqualsInsideToe = insideToe2d.epsilonEquals(adjustedSupportPolygon.getNextFrameVertex(i), epsilon);
         boolean nextEqualsOutsideToe = outsideToe2d.epsilonEquals(adjustedSupportPolygon.getNextFrameVertex(i), epsilon);

         if (equalsInsideToe)
         {
            if (prevEqualsOutsideToe)
            {
               connectingEdgeA = new FrameLine2d(insideToe2d, adjustedSupportPolygon.getNextFrameVertex(i));
            }
            else if (nextEqualsOutsideToe)
            {
               connectingEdgeA = new FrameLine2d(insideToe2d, adjustedSupportPolygon.getPreviousFrameVertex(i));
            }
            else
            {
               connectingEdgeA = new FrameLine2d(insideToe2d, adjustedSupportPolygon.getPreviousFrameVertex(i));
               connectingEdgeB = new FrameLine2d(insideToe2d, adjustedSupportPolygon.getNextFrameVertex(i));
               placeVTPOnInsideToe = true;
            }
         }
         else if (equalsOutsideToe)
         {
            if (prevEqualsInsideToe)
            {
               connectingEdgeB = new FrameLine2d(outsideToe2d, adjustedSupportPolygon.getNextFrameVertex(i));
            }
            else if (nextEqualsInsideToe)
            {
               connectingEdgeB = new FrameLine2d(outsideToe2d, adjustedSupportPolygon.getPreviousFrameVertex(i));
            }
            else
            {
               connectingEdgeA = new FrameLine2d(outsideToe2d, adjustedSupportPolygon.getNextFrameVertex(i));
               connectingEdgeB = new FrameLine2d(outsideToe2d, adjustedSupportPolygon.getPreviousFrameVertex(i));
               placeVTPOnOutsideToe = true;
            }
         }

         prevEqualsInsideToe = equalsInsideToe;
         prevEqualsOutsideToe = equalsOutsideToe;
         equalsInsideToe = nextEqualsInsideToe;
         equalsOutsideToe = nextEqualsOutsideToe;
      }

      if (connectingEdgeA == null || connectingEdgeB == null)
      {
         throw new RuntimeException("Cannot find connecting edges.");
      }
      FramePoint2d edgeIntersection = connectingEdgeA.intersectionWith(connectingEdgeB);
      copDesired.changeFrame(midFeetZUpFrame);
      adjustedSupportPolygon.orthogonalProjection(copDesired);

      FrameLine2d controlLine = new FrameLine2d(edgeIntersection, copDesired);

      FramePoint2d[] intersections = footPolygonInMidFeetZUp.intersectionWith(controlLine);

      FramePoint2d upcomingSupportFootVTP;
      if (intersections == null)
      {
         upcomingSupportFootVTP = footPolygonInMidFeetZUp.getClosestVertexCopy(controlLine);
      }
      else if (intersections.length == 1)
      {
         upcomingSupportFootVTP = intersections[0];
      }
      else
      {
         FrameLineSegment2d footLine = new FrameLineSegment2d(intersections);
         upcomingSupportFootVTP = footLine.pointBetweenEndPointsGivenParameter(0.5);
      }

      FramePoint2d toePoint2d;
      if (placeVTPOnInsideToe)
      {
         toePoint2d = insideToe2d;
      }
      else if (placeVTPOnOutsideToe)
      {
         toePoint2d = outsideToe2d;
      }
      else
      {
         toePoint2d = toeLine.intersectionWith(controlLine);
      }

      FrameLineSegment2d vtpToVTP = new FrameLineSegment2d(upcomingSupportFootVTP, toePoint2d);
      if (!vtpToVTP.isBetweenEndpoints(copDesired, 1.0e-3))
      {
         if (copDesired.distance(upcomingSupportFootVTP) < copDesired.distance(toePoint2d))
         {
            upcomingSupportFootVTP = copDesired;
         }
      }

      virtualToePoints.set(upcomingSupportLeg, upcomingSupportFootVTP);
      virtualToePoints.set(upcomingSwingLeg, toePoint2d);

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePoint vtp3D = virtualToePoints.get(robotSide).toFramePoint();
         vtp3D.changeFrame(worldFrame);
         virtualToePointsWorld.get(robotSide).set(vtp3D);
      }

   }

   public void hideVisualizationGraphics()
   {
      vtpConvexPolygon.setFrameConvexPolygon2d(null);
      for (RobotSide robotSide : RobotSide.values)
      {
         virtualToePointsWorld.get(robotSide).setToNaN();
      }
   }

   public void setupForDebugViz(boolean debugViz, boolean removeDebugVizEachTime)
   {
      System.err.println("Not implemented!");

   }

   public void packVirtualToePoints(SideDependentList<FramePoint2d> originalVirtualToePoints, FramePoint2d copDesired,
         FrameConvexPolygon2dAndConnectingEdges supportPolygonAndEdges, RobotSide upcomingSupportSide)
   {
      throw new RuntimeException("Not implemented!");

   }

}
