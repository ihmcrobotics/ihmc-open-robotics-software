package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameLineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.robotics.geometry.StringStretcher2d;
import us.ihmc.robotics.math.trajectories.YoConfigurablePolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;

public class BetterLookAheadCoMHeightTrajectoryGenerator
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private boolean visualize = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoConfigurablePolynomial spline = new YoConfigurablePolynomial("height", 8, registry);

   private final YoDouble minimumHeightAboveGround = new YoDouble("minimumHeightAboveGround", registry);
   private final YoDouble nominalHeightAboveGround = new YoDouble("nominalHeightAboveGround", registry);
   private final YoDouble maximumHeightAboveGround = new YoDouble("maximumHeightAboveGround", registry);

   private final YoDouble doubleSupportPercentageIn = new YoDouble("doubleSupportPercentageIn", registry);
   private final YoDouble percentageThroughSegment = new YoDouble("percentageThroughSegment", registry);
   private final YoDouble splineQuery = new YoDouble("splineQuery", registry);

   private final YoFramePoint3D transferFromPosition = new YoFramePoint3D("transferFromPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D transferToPosition = new YoFramePoint3D("transferToPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D desiredCoMPositionAtStart = new YoFramePoint3D("desiredCoMPositionAtStart", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble desiredCoMHeight = new YoDouble("desiredCoMHeight", registry);
   private final YoFramePoint3D queryPosition = new YoFramePoint3D("queryPosition", ReferenceFrame.getWorldFrame(), registry);

   private final FrameLineSegment3D projectionSegmentInWorld = new FrameLineSegment3D();

   private final YoFramePoint3D contactFrameZeroPosition = new YoFramePoint3D("contactFrameZeroPosition", worldFrame, registry);
   private final YoFramePoint3D contactFrameOnePosition = new YoFramePoint3D("contactFrameOnePosition", worldFrame, registry);

   private final YoFramePoint3D startWaypointInWorld, startWaypointMinInWorld, startWaypointMaxInWorld;
   private final YoFramePoint3D firstMidpointInWorld, firstMidpointMinInWorld, firstMidpointMaxInWorld;
   private final YoFramePoint3D secondMidpointInWorld, secondMidpointMinInWorld, secondMidpointMaxInWorld;
   private final YoFramePoint3D endWaypointInWorld, endWaypointMinInWorld, endWaypointMaxInWorld;

   private final BagOfBalls bagOfBalls;

   private final DoubleProvider yoTime;

   private ReferenceFrame frameOfLastFootstep;
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final FramePoint3D startWaypointMinimum = new FramePoint3D();
   private final FramePoint3D firstMidpointMinimum = new FramePoint3D();
   private final FramePoint3D secondMidpointMinimum = new FramePoint3D();
   private final FramePoint3D endWaypointMinimum = new FramePoint3D();

   private final FramePoint3D startWaypointNominal = new FramePoint3D();
   private final FramePoint3D firstMidpointNominal = new FramePoint3D();
   private final FramePoint3D secondMidpointNominal = new FramePoint3D();
   private final FramePoint3D endWaypointNominal = new FramePoint3D();

   private final FramePoint3D startWaypointMaximum = new FramePoint3D();
   private final FramePoint3D firstMidpointMaximum = new FramePoint3D();
   private final FramePoint3D secondMidpointMaximum = new FramePoint3D();
   private final FramePoint3D endWaypointMaximum = new FramePoint3D();

   private final FramePoint3D startWaypoint = new FramePoint3D();
   private final FramePoint3D firstMidpoint = new FramePoint3D();
   private final FramePoint3D secondMidpoint = new FramePoint3D();
   private final FramePoint3D endWaypoint = new FramePoint3D();

   private final FramePoint3D tempFramePointForViz1 = new FramePoint3D();

   private final FramePoint3D com = new FramePoint3D();

   private final CoMHeightPartialDerivativesData comHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
   private final FramePoint3D tempFramePoint = new FramePoint3D();

   private final FramePoint3D startCoMPosition = new FramePoint3D();
   private final FramePoint3D endCoMPosition = new FramePoint3D();

   public BetterLookAheadCoMHeightTrajectoryGenerator(double minimumHeightAboveGround,
                                                      double nominalHeightAboveGround,
                                                      double maximumHeightAboveGround,
                                                      double defaultOffsetHeightAboveGround,
                                                      double doubleSupportPercentageIn,
                                                      ReferenceFrame centerOfMassFrame,
                                                      SideDependentList<? extends ReferenceFrame> soleFrames,
                                                      DoubleProvider yoTime,
                                                      YoGraphicsListRegistry yoGraphicsListRegistry,
                                                      YoVariableRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.soleFrames = soleFrames;
      frameOfLastFootstep = soleFrames.get(RobotSide.LEFT);
      this.yoTime = yoTime;

      setMinimumHeightAboveGround(minimumHeightAboveGround);
      setNominalHeightAboveGround(nominalHeightAboveGround);
      setMaximumHeightAboveGround(maximumHeightAboveGround);

      this.doubleSupportPercentageIn.set(doubleSupportPercentageIn);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry == null)
         visualize = false;

      startWaypointInWorld = new YoFramePoint3D("startWaypointInWorld", worldFrame, registry);
      startWaypointMinInWorld = new YoFramePoint3D("startWaypointMinInWorld", worldFrame, registry);
      startWaypointMaxInWorld = new YoFramePoint3D("startWaypointMaxInWorld", worldFrame, registry);

      firstMidpointInWorld = new YoFramePoint3D("firstMidpointInWorld", worldFrame, registry);
      firstMidpointMinInWorld = new YoFramePoint3D("firstMidpointMinInWorld", worldFrame, registry);
      firstMidpointMaxInWorld = new YoFramePoint3D("firstMidpointMaxInWorld", worldFrame, registry);

      secondMidpointInWorld = new YoFramePoint3D("secondMidpointInWorld", worldFrame, registry);
      secondMidpointMinInWorld = new YoFramePoint3D("secondMidpointMinInWorld", worldFrame, registry);
      secondMidpointMaxInWorld = new YoFramePoint3D("secondMidpointMaxInWorld", worldFrame, registry);

      endWaypointInWorld = new YoFramePoint3D("endWaypointInWorld", worldFrame, registry);
      endWaypointMinInWorld = new YoFramePoint3D("endWaypointMinInWorld", worldFrame, registry);
      endWaypointMaxInWorld = new YoFramePoint3D("endWaypointMaxInWorld", worldFrame, registry);

      if (visualize)
      {
         double pointSize = 0.03;

         String prefix = "better_";

         AppearanceDefinition startColor = YoAppearance.CadetBlue();
         AppearanceDefinition firstMidpointColor = YoAppearance.Chartreuse();
         AppearanceDefinition secondMidpointColor = YoAppearance.BlueViolet();
         AppearanceDefinition endColor = YoAppearance.Azure();

         YoGraphicPosition position0 = new YoGraphicPosition(prefix + "contactFrame0", contactFrameZeroPosition, pointSize, YoAppearance.Purple());
         YoGraphicPosition position1 = new YoGraphicPosition(prefix + "contactFrame1", contactFrameOnePosition, pointSize, YoAppearance.Orange());

         YoGraphicPosition pointS0Viz = new YoGraphicPosition(prefix + "StartWaypoint", startWaypointInWorld, pointSize, startColor);
         YoGraphicPosition pointSFViz = new YoGraphicPosition(prefix + "FirstMidpoint", firstMidpointInWorld, pointSize, firstMidpointColor);
         YoGraphicPosition pointD0Viz = new YoGraphicPosition(prefix + "SecondMidpoint", secondMidpointInWorld, pointSize, secondMidpointColor);
         YoGraphicPosition pointDFViz = new YoGraphicPosition(prefix + "EndWaypoint", endWaypointInWorld, pointSize, endColor);

         YoGraphicPosition pointS0MinViz = new YoGraphicPosition(prefix + "StartWaypointMin", startWaypointMinInWorld, 0.8 * pointSize, startColor);
         YoGraphicPosition pointSFMinViz = new YoGraphicPosition(prefix + "FirstMidpointMin", firstMidpointMinInWorld, 0.8 * pointSize, firstMidpointColor);
         YoGraphicPosition pointD0MinViz = new YoGraphicPosition(prefix + "SecondMidpointMin", secondMidpointMinInWorld, 0.8 * pointSize, secondMidpointColor);
         YoGraphicPosition pointDFMinViz = new YoGraphicPosition(prefix + "EndWaypointMin", endWaypointMinInWorld, 0.8 * pointSize, endColor);

         YoGraphicPosition pointS0MaxViz = new YoGraphicPosition(prefix + "StartWaypointMax", startWaypointMaxInWorld, 0.9 * pointSize, startColor);
         YoGraphicPosition pointSFMaxViz = new YoGraphicPosition(prefix + "FirstMidpointMax", firstMidpointMaxInWorld, 0.9 * pointSize, firstMidpointColor);
         YoGraphicPosition pointD0MaxViz = new YoGraphicPosition(prefix + "SecondMidpointMax", secondMidpointMaxInWorld, 0.9 * pointSize, secondMidpointColor);
         YoGraphicPosition pointDFMaxViz = new YoGraphicPosition(prefix + "EndWaypointMax", endWaypointMaxInWorld, 0.9 * pointSize, endColor);

         bagOfBalls = new BagOfBalls(15, 0.01, "height", registry, yoGraphicsListRegistry);

         YoGraphicPosition desiredCoMPositionViz = new YoGraphicPosition(prefix + "desiredCoMPosition",
                                                                         desiredCoMPosition,
                                                                         1.1 * pointSize,
                                                                         YoAppearance.Gold());

         String graphicListName = "BetterCoMHeightTrajectoryGenerator";
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, position0);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, position1);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0Viz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0Viz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFViz);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0MinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFMinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MinViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFMinViz);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointS0MaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointSFMaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointD0MaxViz);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, pointDFMaxViz);
         //
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, desiredCoMPositionViz);
      }
      else
      {
         bagOfBalls = null;
      }
   }

   public void reset()
   {
      tempFramePoint.setToZero(centerOfMassFrame);
      tempFramePoint.changeFrame(worldFrame);

      desiredCoMPosition.set(tempFramePoint);
   }

   public void setMinimumHeightAboveGround(double minimumHeightAboveGround)
   {
      this.minimumHeightAboveGround.set(minimumHeightAboveGround);
   }

   public void setNominalHeightAboveGround(double nominalHeightAboveGround)
   {
      this.nominalHeightAboveGround.set(nominalHeightAboveGround);
   }

   public void setMaximumHeightAboveGround(double maximumHeightAboveGround)
   {
      this.maximumHeightAboveGround.set(maximumHeightAboveGround);
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      frameOfLastFootstep = soleFrames.get(supportLeg);
   }

   public void initialize(NewTransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      FramePoint3DReadOnly transferToFootstepPosition = transferToAndNextFootstepsData.getTransferToPosition();

      tempFramePoint.setToZero(frameOfLastFootstep);
      transferFromPosition.setMatchingFrame(tempFramePoint);
      transferToPosition.setMatchingFrame(transferToFootstepPosition);
      desiredCoMPositionAtStart.set(desiredCoMPosition);

      startCoMPosition.setIncludingFrame(desiredCoMPosition);
      startCoMPosition.changeFrame(frameOfLastFootstep);

      endCoMPosition.setIncludingFrame(transferToFootstepPosition);
      endCoMPosition.changeFrame(frameOfLastFootstep);
      double endAnkleZ = endCoMPosition.getZ();

      endCoMPosition.addZ(nominalHeightAboveGround.getDoubleValue());

      double midstanceWidth = 0.5 * endCoMPosition.getY();

      startCoMPosition.setY(midstanceWidth);
      endCoMPosition.setY(midstanceWidth);
      endCoMPosition.setY(midstanceWidth);

      setWaypointFrames(frameOfLastFootstep);

      double startX, startZ, endX, endZ;
      if (startCoMPosition.getX() < endCoMPosition.getX())
      {
         startX = startCoMPosition.getX();
         startZ = startCoMPosition.getZ();
         endX = endCoMPosition.getX();
         endZ = endCoMPosition.getZ();
      }
      else
      {
         endX = startCoMPosition.getX();
         endZ = startCoMPosition.getZ();
         startX = endCoMPosition.getX();
         startZ = endCoMPosition.getZ();
      }

      double firstMidpointX = InterpolationTools.linearInterpolate(startX, endX, doubleSupportPercentageIn.getDoubleValue());
      double secondMidpointX = InterpolationTools.linearInterpolate(endX, startX, doubleSupportPercentageIn.getDoubleValue());

      startWaypoint.set(startX, midstanceWidth, startZ);
      endWaypoint.set(endX, midstanceWidth, endZ);
      firstMidpoint.setY(midstanceWidth);
      secondMidpoint.setY(midstanceWidth);

      startWaypointMinimum.set(0.0, midstanceWidth, minimumHeightAboveGround.getDoubleValue());
      startWaypointNominal.set(0.0, midstanceWidth, nominalHeightAboveGround.getDoubleValue());
      startWaypointMaximum.set(0.0, midstanceWidth, maximumHeightAboveGround.getDoubleValue());

      endWaypointMinimum.set(endX, midstanceWidth, minimumHeightAboveGround.getDoubleValue() + endAnkleZ);
      endWaypointNominal.set(endX, midstanceWidth, nominalHeightAboveGround.getDoubleValue() + endAnkleZ);
      endWaypointMaximum.set(endX, midstanceWidth, maximumHeightAboveGround.getDoubleValue() + endAnkleZ);

      firstMidpointMinimum.set(firstMidpointX, midstanceWidth, findWaypointHeight(minimumHeightAboveGround.getDoubleValue(), startX, firstMidpointX, 0.0));
      firstMidpointNominal.set(firstMidpointX, midstanceWidth, findWaypointHeight(nominalHeightAboveGround.getDoubleValue(), startX, firstMidpointX, 0.0));
      firstMidpointMaximum.set(firstMidpointX,
                               midstanceWidth,
                               findWaypointHeight(maximumHeightAboveGround.getDoubleValue(), startX, firstMidpointX, extraToeOffHeight));

      secondMidpointMinimum.set(secondMidpointX,
                                midstanceWidth,
                                findWaypointHeight(minimumHeightAboveGround.getDoubleValue(), endX, secondMidpointX, endAnkleZ));
      secondMidpointNominal.set(secondMidpointX,
                                midstanceWidth,
                                findWaypointHeight(nominalHeightAboveGround.getDoubleValue(), endX, secondMidpointX, endAnkleZ));
      secondMidpointMaximum.set(secondMidpointX,
                                midstanceWidth,
                                findWaypointHeight(maximumHeightAboveGround.getDoubleValue(), endX, secondMidpointX, endAnkleZ));

      computeHeightsToUseByStretchingString();

      spline.reshape(4);
      spline.addPositionConstraint(startWaypoint.getX(), startWaypoint.getZ());
      spline.addPositionConstraint(firstMidpoint.getX(), firstMidpoint.getZ());
      spline.addPositionConstraint(secondMidpoint.getX(), secondMidpoint.getZ());
      spline.addPositionConstraint(endWaypoint.getX(), endWaypoint.getZ());
      spline.solve();

      contactFrameZeroPosition.setMatchingFrame(startWaypoint);
      contactFrameOnePosition.setMatchingFrame(endWaypoint);

      // FIXME the projection segment is real wrong.
      projectionSegmentInWorld.getFirstEndpoint().set(contactFrameZeroPosition);
      projectionSegmentInWorld.getSecondEndpoint().set(contactFrameOnePosition);

      startWaypointInWorld.setMatchingFrame(startWaypoint);
      startWaypointMinInWorld.setMatchingFrame(startWaypointMinimum);
      startWaypointMaxInWorld.setMatchingFrame(startWaypointMaximum);

      firstMidpointInWorld.setMatchingFrame(firstMidpoint);
      firstMidpointMinInWorld.setMatchingFrame(firstMidpointMinimum);
      firstMidpointMaxInWorld.setMatchingFrame(firstMidpointMaximum);

      secondMidpointInWorld.setMatchingFrame(secondMidpoint);
      secondMidpointMinInWorld.setMatchingFrame(secondMidpointMinimum);
      secondMidpointMaxInWorld.setMatchingFrame(secondMidpointMaximum);

      endWaypointInWorld.setMatchingFrame(endWaypoint);
      endWaypointMinInWorld.setMatchingFrame(endWaypointMinimum);
      endWaypointMaxInWorld.setMatchingFrame(endWaypointMaximum);

      if (visualize)
      {
         bagOfBalls.reset();
         int numberOfPoints = bagOfBalls.getNumberOfBalls();

         for (int i = 0; i < numberOfPoints; i++)
         {
            double alpha = ((double) (i + 1)) / ((double) numberOfPoints);
            tempFramePointForViz1.setToZero(worldFrame);
            tempFramePointForViz1.interpolate(contactFrameZeroPosition, contactFrameOnePosition, alpha);

            this.solve(comHeightPartialDerivativesData, tempFramePointForViz1, false);

            tempFramePointForViz1.checkReferenceFrameMatch(comHeightPartialDerivativesData.getFrameOfCoMHeight());
            tempFramePointForViz1.setZ(comHeightPartialDerivativesData.getComHeight());

            bagOfBalls.setBallLoop(tempFramePointForViz1);
         }
      }
   }

   private void setWaypointFrames(ReferenceFrame referenceFrame)
   {
      startWaypoint.setToZero(referenceFrame);
      firstMidpoint.setToZero(referenceFrame);
      secondMidpoint.setToZero(referenceFrame);
      endWaypoint.setToZero(referenceFrame);

      startWaypointMinimum.setToZero(referenceFrame);
      firstMidpointMinimum.setToZero(referenceFrame);
      secondMidpointMinimum.setToZero(referenceFrame);
      endWaypointMinimum.setToZero(referenceFrame);

      startWaypointNominal.setToZero(referenceFrame);
      firstMidpointNominal.setToZero(referenceFrame);
      secondMidpointNominal.setToZero(referenceFrame);
      endWaypointNominal.setToZero(referenceFrame);

      startWaypointMaximum.setToZero(referenceFrame);
      firstMidpointMaximum.setToZero(referenceFrame);
      secondMidpointMaximum.setToZero(referenceFrame);
      endWaypointMaximum.setToZero(referenceFrame);
   }

   private final StringStretcher2d stringStretcher = new StringStretcher2d();
   private final List<Point2DBasics> stretchedStringWaypoints = new ArrayList<>();

   private void computeHeightsToUseByStretchingString()
   {
      stringStretcher.reset();
      // startWaypoint is at previous
      stringStretcher.setStartPoint(startWaypoint.getX(), startWaypoint.getZ());

      double sFNomHeight = endWaypoint.getZ();

      // If the final single support nominal position is higher than the final double support max position, decrease it
      // but don't decrease it below the final single support minimum height.
      // FIXME this is weird
      if (sFNomHeight > secondMidpointMaximum.getZ())
      {
         sFNomHeight = Math.max(secondMidpointMaximum.getZ(), sFNomHeight);
         stringStretcher.setEndPoint(endWaypoint.getX(), sFNomHeight);
      }
      else
      {
         stringStretcher.setEndPoint(endWaypoint.getX(), endWaypoint.getZ());
      }

      //      stringStretcher.addMinMaxPoints(0.0, minimumHeightAboveGround.getDoubleValue(), 0.0, maximumHeightAboveGround.getDoubleValue());
      stringStretcher.addMinMaxPoints(firstMidpointMinimum.getX(), firstMidpointMinimum.getZ(), firstMidpointMaximum.getX(), firstMidpointMaximum.getZ());
      stringStretcher.addMinMaxPoints(secondMidpointMinimum.getX(), secondMidpointMinimum.getZ(), secondMidpointMaximum.getX(), secondMidpointMaximum.getZ());

      stringStretcher.stretchString(stretchedStringWaypoints);

      firstMidpoint.setX(stretchedStringWaypoints.get(1).getX());
      firstMidpoint.setZ(stretchedStringWaypoints.get(1).getY());

      secondMidpoint.setX(stretchedStringWaypoints.get(2).getX());
      secondMidpoint.setZ(stretchedStringWaypoints.get(2).getY());

      endWaypoint.setX(stretchedStringWaypoints.get(3).getX());
      endWaypoint.setZ(stretchedStringWaypoints.get(3).getY());
   }

   private static double findWaypointHeight(double desiredDistanceFromFoot, double startAnkleX, double queryX, double extraToeOffHeight)
   {
      return Math.sqrt(MathTools.square(desiredDistanceFromFoot) - MathTools.square(queryX - startAnkleX)) + extraToeOffHeight;
   }

   public void solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, boolean isInDoubleSupport)
   {
      com.setToZero(centerOfMassFrame);
      com.changeFrame(worldFrame);

      solve(comHeightPartialDerivativesDataToPack, com, isInDoubleSupport);

      desiredCoMPosition.set(com.getX(), com.getY(), comHeightPartialDerivativesDataToPack.getComHeight());
   }

   private void solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, FramePoint3DBasics queryPoint, boolean isInDoubleSupport)
   {
      this.queryPosition.set(queryPoint);
      // TODO this is not how we want to do this, we want to be using the actual center of mass point
      projectionSegmentInWorld.orthogonalProjection(queryPoint);

      double percentAlongSegment = projectionSegmentInWorld.percentageAlongLineSegment(queryPoint);
      percentageThroughSegment.set(percentAlongSegment);

      double splineQuery = InterpolationTools.linearInterpolate(startWaypoint.getX(), endWaypoint.getX(), percentAlongSegment);

      // Happens when the robot gets stuck in double support but the ICP is still being dragged in the front support foot.
      //      if (isInDoubleSupport)
      //         splineQuery = Math.min(splineQuery, secondMidpoint.getX());

      this.splineQuery.set(splineQuery);
      spline.compute(splineQuery);
      //      curve.compute(splineQuery);

      double z = spline.getPosition();
      //      double z = curve.getPosition();
      double dzds = spline.getVelocity();
      double ddzdds = spline.getAcceleration();

      double length = startWaypoint.distance(endWaypoint);
      double dsdx = (endWaypoint.getX() - startWaypoint.getX()) / length;
      double dsdy = (endWaypoint.getZ() - startWaypoint.getZ()) / length;

      double ddsddx = 0;
      double ddsddy = 0;
      double ddsdxdy = 0;

      double dzdx = dsdx * dzds;
      double dzdy = dsdy * dzds;
      double ddzddx = dzds * ddsddx + ddzdds * dsdx * dsdx;
      double ddzddy = dzds * ddsddy + ddzdds * dsdy * dsdy;
      double ddzdxdy = ddzdds * dsdx * dsdy + dzds * ddsdxdy;

      tempFramePoint.setIncludingFrame(frameOfLastFootstep, 0.0, 0.0, z);
      tempFramePoint.changeFrame(worldFrame);
      comHeightPartialDerivativesDataToPack.setCoMHeight(worldFrame, tempFramePoint.getZ());
      comHeightPartialDerivativesDataToPack.setPartialDzDx(dzdx);
      comHeightPartialDerivativesDataToPack.setPartialDzDy(dzdy);
      comHeightPartialDerivativesDataToPack.setPartialD2zDxDy(ddzdxdy);
      comHeightPartialDerivativesDataToPack.setPartialD2zDx2(ddzddx);
      comHeightPartialDerivativesDataToPack.setPartialD2zDy2(ddzddy);

      desiredCoMHeight.set(z);
   }

   public void goHome(double trajectoryTime)
   {
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
   }

   public void initializeDesiredHeightToCurrent()
   {
   }

   public void getCurrentDesiredHeight(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(desiredCoMPosition);
   }

   public double getOffsetHeightTimeInTrajectory()
   {
      return yoTime.getValue();
   }
}
