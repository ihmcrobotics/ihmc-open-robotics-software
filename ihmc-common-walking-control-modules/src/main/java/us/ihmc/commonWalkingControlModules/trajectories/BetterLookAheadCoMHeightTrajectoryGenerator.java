package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameLineSegment3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
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

   private final YoBoolean useOnlyFirstSpline = new YoBoolean("useOnlyFirstSpline", registry);

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

   private final BagOfBalls bagOfBalls;

   private final DoubleProvider yoTime;

   private ReferenceFrame frameOfLastFootstep;
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final CoMHeightTrajectoryWaypoint startWaypoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint firstMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint secondMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint thirdMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint middleWaypoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint fourthMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint fifthMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint sixthMidpoint = new CoMHeightTrajectoryWaypoint();
   private final CoMHeightTrajectoryWaypoint endWaypoint = new CoMHeightTrajectoryWaypoint();

   private final List<CoMHeightTrajectoryWaypoint> waypoints = new ArrayList<>();

   private final FramePoint3D tempFramePointForViz1 = new FramePoint3D();

   private final FramePoint3D com = new FramePoint3D();

   private final CoMHeightPartialDerivativesData comHeightPartialDerivativesData = new CoMHeightPartialDerivativesData();
   private final FramePoint3D tempFramePoint = new FramePoint3D();

   private final FramePoint3D startCoMPosition = new FramePoint3D();
   private final FramePoint3D middleCoMPosition = new FramePoint3D();
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

      startWaypoint.createYoVariables("startWaypoint", registry);
      firstMidpoint.createYoVariables("firstMidpoint", registry);
      secondMidpoint.createYoVariables("secondMidpoint", registry);
      thirdMidpoint.createYoVariables("thirdMidpoint", registry);
      middleWaypoint.createYoVariables("middleWaypoint", registry);
      fourthMidpoint.createYoVariables("fourthMidpoint", registry);
      fifthMidpoint.createYoVariables("fifthMidpoint", registry);
      sixthMidpoint.createYoVariables("sixthMidpoint", registry);
      endWaypoint.createYoVariables("endWaypoint", registry);

      if (visualize && yoGraphicsListRegistry != null)
      {
         double pointSize = 0.03;

         String prefix = "better_";

         AppearanceDefinition startColor = YoAppearance.CadetBlue();
         AppearanceDefinition firstMidpointColor = YoAppearance.Chartreuse();
         AppearanceDefinition secondMidpointColor = YoAppearance.BlueViolet();
         AppearanceDefinition middleColor = YoAppearance.Azure();

         YoGraphicPosition position0 = new YoGraphicPosition(prefix + "contactFrame0", contactFrameZeroPosition, pointSize, YoAppearance.Purple());
         YoGraphicPosition position1 = new YoGraphicPosition(prefix + "contactFrame1", contactFrameOnePosition, pointSize, YoAppearance.Orange());
         YoGraphicPosition desiredCoMPositionViz = new YoGraphicPosition(prefix + "desiredCoMPosition",
                                                                         desiredCoMPosition,
                                                                         1.1 * pointSize,
                                                                         YoAppearance.Gold());

         String graphicListName = "BetterCoMHeightTrajectoryGenerator";

         startWaypoint.setupViz(graphicListName, prefix + "StartWaypoint", startColor, yoGraphicsListRegistry);
         firstMidpoint.setupViz(graphicListName, prefix + "FirstMidpoint", firstMidpointColor, yoGraphicsListRegistry);
         secondMidpoint.setupViz(graphicListName, prefix + "SecondMidpoint", secondMidpointColor, yoGraphicsListRegistry);
         thirdMidpoint.setupViz(graphicListName, prefix + "ThirdMidpoint", secondMidpointColor, yoGraphicsListRegistry);
         middleWaypoint.setupViz(graphicListName, prefix + "MiddleWaypoint", middleColor, yoGraphicsListRegistry);

         bagOfBalls = new BagOfBalls(15, 0.01, "height", registry, yoGraphicsListRegistry);

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, position0);
         yoGraphicsListRegistry.registerYoGraphic(graphicListName, position1);
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
      FramePoint3DReadOnly nextFootstepPosition = transferToAndNextFootstepsData.getNextFootstepPosition();

      boolean hasNextFootstep = !nextFootstepPosition.containsNaN() && !useOnlyFirstSpline.getBooleanValue();

      tempFramePoint.setToZero(frameOfLastFootstep);
      transferFromPosition.setMatchingFrame(tempFramePoint);
      transferToPosition.setMatchingFrame(transferToFootstepPosition);
      desiredCoMPositionAtStart.set(desiredCoMPosition);

      startCoMPosition.setIncludingFrame(desiredCoMPosition);
      startCoMPosition.changeFrame(frameOfLastFootstep);

      middleCoMPosition.setIncludingFrame(transferToFootstepPosition);
      middleCoMPosition.changeFrame(frameOfLastFootstep);
      double middleAnkleZ = middleCoMPosition.getZ();
      middleCoMPosition.addZ(nominalHeightAboveGround.getDoubleValue());

      endCoMPosition.setIncludingFrame(nextFootstepPosition);
      endCoMPosition.changeFrame(frameOfLastFootstep);
      double endAnkleZ = endCoMPosition.getZ();
      endCoMPosition.addZ(nominalHeightAboveGround.getDoubleValue());

      double midstanceWidth = 0.5 * middleCoMPosition.getY();

      startCoMPosition.setY(midstanceWidth);
      middleCoMPosition.setY(midstanceWidth);
      endCoMPosition.setY(midstanceWidth);

      setWaypointFrames(frameOfLastFootstep);

      double start1X, start1Z, end1X, end1Z;
      boolean isStart1TheBeginning;
      if (startCoMPosition.getX() < middleCoMPosition.getX())
      {
         start1X = startCoMPosition.getX();
         start1Z = startCoMPosition.getZ();
         end1X = middleCoMPosition.getX();
         end1Z = middleCoMPosition.getZ();
         isStart1TheBeginning = true;
      }
      else
      {
         end1X = startCoMPosition.getX();
         end1Z = startCoMPosition.getZ();
         start1X = middleCoMPosition.getX();
         start1Z = middleCoMPosition.getZ();
         isStart1TheBeginning = false;
      }

      double dzds1 = (middleCoMPosition.getZ() - startCoMPosition.getZ()) / (middleCoMPosition.getX() - startCoMPosition.getX());
      double dzds2 = (endCoMPosition.getZ() - middleCoMPosition.getZ()) / (endCoMPosition.getX() - middleCoMPosition.getX());
      double jointSlope = 0.5 * (dzds1 + dzds2);

      double firstMidpointX = InterpolationTools.linearInterpolate(start1X, end1X, doubleSupportPercentageIn.getDoubleValue());
      double secondMidpointX = InterpolationTools.linearInterpolate(start1X, end1X, 0.5);
      double thirdMidpointX = InterpolationTools.linearInterpolate(end1X, start1X, doubleSupportPercentageIn.getDoubleValue());

      startWaypoint.setXY(start1X, midstanceWidth);
      firstMidpoint.setXY(firstMidpointX, midstanceWidth);
      secondMidpoint.setXY(secondMidpointX, midstanceWidth);
      thirdMidpoint.setXY(thirdMidpointX, midstanceWidth);
      middleWaypoint.setXY(end1X, midstanceWidth);

      double secondMinHeight = Math.max(findWaypointHeight(minimumHeightAboveGround.getDoubleValue(), start1X, secondMidpointX, 0.0),
                                        findWaypointHeight(minimumHeightAboveGround.getDoubleValue(), end1X, secondMidpointX, middleAnkleZ));
      double secondMaxHeight = Math.min(findWaypointHeight(maximumHeightAboveGround.getDoubleValue(), start1X, secondMidpointX, extraToeOffHeight),
                                        findWaypointHeight(maximumHeightAboveGround.getDoubleValue(), end1X, secondMidpointX, middleAnkleZ));
      startWaypoint.setMinMax(minimumHeightAboveGround.getDoubleValue(), maximumHeightAboveGround.getDoubleValue());
      firstMidpoint.setMinMax(findWaypointHeight(minimumHeightAboveGround.getDoubleValue(), start1X, firstMidpointX, 0.0),
                              findWaypointHeight(maximumHeightAboveGround.getDoubleValue(), start1X, firstMidpointX, extraToeOffHeight));
      secondMidpoint.setMinMax(secondMinHeight, secondMaxHeight);
      thirdMidpoint.setMinMax(findWaypointHeight(minimumHeightAboveGround.getDoubleValue(), end1X, thirdMidpointX, middleAnkleZ),
                              findWaypointHeight(maximumHeightAboveGround.getDoubleValue(), end1X, thirdMidpointX, middleAnkleZ));
      middleWaypoint.setMinMax(minimumHeightAboveGround.getDoubleValue() + middleAnkleZ, maximumHeightAboveGround.getDoubleValue() + middleAnkleZ);

      waypoints.clear();
      waypoints.add(startWaypoint);
      waypoints.add(firstMidpoint);
      waypoints.add(thirdMidpoint);
      waypoints.add(secondMidpoint);
      waypoints.add(middleWaypoint);

      computeHeightsToUseByStretchingString(start1Z, end1Z, waypoints);

      if (hasNextFootstep)
      {
         spline.reshape(waypoints.size() + 1);
         if (isStart1TheBeginning)
            spline.addVelocityConstraint(waypoints.get(waypoints.size() - 1).getX(), jointSlope);
         else
            spline.addVelocityConstraint(waypoints.get(0).getX(), jointSlope);
      }
      else
      {
         spline.reshape(waypoints.size());
      }
      for (int i = 0; i < waypoints.size(); i++)
         spline.addPositionConstraint(waypoints.get(i).getX(), waypoints.get(i).getHeight());
      spline.solve();

      contactFrameZeroPosition.setMatchingFrame(waypoints.get(0).getWaypoint());
      contactFrameOnePosition.setMatchingFrame(waypoints.get(waypoints.size() - 1).getWaypoint());

      // FIXME the projection segment is real wrong.
      projectionSegmentInWorld.getFirstEndpoint().set(contactFrameZeroPosition);
      projectionSegmentInWorld.getSecondEndpoint().set(contactFrameOnePosition);

      for (int i = 0; i < waypoints.size(); i++)
      {
         waypoints.get(i).update();
      }

      if (visualize)
      {
         bagOfBalls.reset();
         int numberOfPoints = bagOfBalls.getNumberOfBalls();

         for (int i = 0; i < numberOfPoints; i++)
         {
            double alpha = ((double) (i + 1)) / ((double) numberOfPoints);

            projectionSegmentInWorld.pointBetweenEndpointsGivenPercentage(alpha, tempFramePointForViz1);
            this.solve(comHeightPartialDerivativesData, tempFramePointForViz1, false);

            tempFramePointForViz1.checkReferenceFrameMatch(comHeightPartialDerivativesData.getFrameOfCoMHeight());
            tempFramePointForViz1.setZ(comHeightPartialDerivativesData.getComHeight());

            bagOfBalls.setBallLoop(tempFramePointForViz1);
         }
      }
   }

   private void solveWaypointSegment()
   {}


   private void setWaypointFrames(ReferenceFrame referenceFrame)
   {
      startWaypoint.setToZero(referenceFrame);
      firstMidpoint.setToZero(referenceFrame);
      secondMidpoint.setToZero(referenceFrame);
      thirdMidpoint.setToZero(referenceFrame);
      middleWaypoint.setToZero(referenceFrame);
      fourthMidpoint.setToZero(referenceFrame);
      fifthMidpoint.setToZero(referenceFrame);
      sixthMidpoint.setToZero(referenceFrame);
      endWaypoint.setToZero(referenceFrame);
   }

   private final StringStretcher2d stringStretcher = new StringStretcher2d();
   private final List<Point2DBasics> stretchedStringWaypoints = new ArrayList<>();

   private void computeHeightsToUseByStretchingString(double startHeight, double endHeight, List<CoMHeightTrajectoryWaypoint> waypoints)
   {
      CoMHeightTrajectoryWaypoint startWaypoint = waypoints.get(0);
      CoMHeightTrajectoryWaypoint lastWaypoint = waypoints.get(waypoints.size() - 1);
      stringStretcher.reset();
      stringStretcher.setStartPoint(startWaypoint.getX(), startHeight);
      stringStretcher.setEndPoint(lastWaypoint.getX(), endHeight);

      for (int i = 1; i < waypoints.size() - 1; i++)
      {
         CoMHeightTrajectoryWaypoint waypoint = waypoints.get(i);
         stringStretcher.addMinMaxPoints(waypoint.getX(), waypoint.getMinHeight(), waypoint.getX(), waypoint.getMaxHeight());
      }

      stringStretcher.stretchString(stretchedStringWaypoints);

      for (int i = 0; i < waypoints.size(); i++)
      {
         CoMHeightTrajectoryWaypoint waypoint = waypoints.get(i);
         waypoint.setX(stretchedStringWaypoints.get(i).getX());
         waypoint.setHeight(stretchedStringWaypoints.get(i).getY());
      }
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

      double splineQuery = InterpolationTools.linearInterpolate(startWaypoint.getX(), middleWaypoint.getX(), percentAlongSegment);

      this.splineQuery.set(splineQuery);
      spline.compute(splineQuery);

      double z = spline.getPosition();
      double dzds = spline.getVelocity();
      double ddzdds = spline.getAcceleration();

      double length = startWaypoint.getWaypoint().distance(middleWaypoint.getWaypoint());
      double dsdx = (middleWaypoint.getX() - startWaypoint.getX()) / length;
      double dsdy = (middleWaypoint.getHeight() - startWaypoint.getHeight()) / length;

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
