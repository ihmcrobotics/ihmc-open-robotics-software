package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
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

   private final YoBoolean isTrajectoryOffsetStopped = new YoBoolean("isOffsetHeightTrajectoryStopped", registry);
   private final YoBoolean initializeToCurrent = new YoBoolean("initializeCoMHeightToCurrent", registry);
   private final BooleanProvider processGoHome = new BooleanParameter("ProcessGoHome", registry, false);

   private final YoDouble offsetHeightAboveGround = new YoDouble("offsetHeightAboveGround", registry);
   private final YoDouble offsetHeightAboveGroundChangedTime = new YoDouble("offsetHeightAboveGroundChangedTime", registry);
   private final YoDouble offsetHeightAboveGroundTrajectoryOutput = new YoDouble("offsetHeightAboveGroundTrajectoryOutput", registry);
   private final YoDouble offsetHeightAboveGroundTrajectoryDuration = new YoDouble("offsetHeightAboveGroundTrajectoryDuration", registry);
   private final MultipleWaypointsTrajectoryGenerator offsetHeightTrajectoryGenerator = new MultipleWaypointsTrajectoryGenerator("pelvisHeightOffset",
                                                                                                                                 registry);

   private final YoLong numberOfQueuedCommands = new YoLong("pelvisHeightNumberOfQueuedCommands", registry);
   private final RecyclingArrayDeque<PelvisHeightTrajectoryCommand> commandQueue = new RecyclingArrayDeque<>(PelvisHeightTrajectoryCommand.class,
                                                                                                             PelvisHeightTrajectoryCommand::set);

   private final YoDouble minimumHeightAboveGround = new YoDouble("minimumHeightAboveGround", registry);
   private final YoDouble nominalHeightAboveGround = new YoDouble("nominalHeightAboveGround", registry);
   private final YoDouble maximumHeightAboveGround = new YoDouble("maximumHeightAboveGround", registry);

   private final YoDouble nominalDoubleSupportPercentageIn = new YoDouble("nominalDoubleSupportPercentageIn", registry);
   private final YoDouble doubleSupportPercentageIn = new YoDouble("doubleSupportPercentageIn", registry);
   private final YoDouble percentageThroughSegment = new YoDouble("percentageThroughSegment", registry);
   private final YoDouble splineQuery = new YoDouble("splineQuery", registry);

   private final YoFramePoint3D transferFromPosition = new YoFramePoint3D("transferFromPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D transferToPosition = new YoFramePoint3D("transferToPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D desiredCoMPositionAtStart = new YoFramePoint3D("desiredCoMPositionAtStart", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble desiredCoMHeight = new YoDouble("desiredCoMHeight", registry);
   private final YoFramePoint3D queryPosition = new YoFramePoint3D("queryPosition", ReferenceFrame.getWorldFrame(), registry);

   private final RecyclingArrayList<CoMHeightTrajectoryWaypoint> heightWaypoints;

   private final DoubleProvider yoTime;

   private ReferenceFrame frameOfLastFootstep;
   private final ReferenceFrame centerOfMassFrame;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final FramePoint3D com = new FramePoint3D();

   private final FramePoint3D tempFramePoint = new FramePoint3D();

   private final FramePoint3D startCoMPosition = new FramePoint3D();
   private final FramePoint3D middleCoMPosition = new FramePoint3D();

   private final SplinedHeightTrajectory splinedHeightTrajectory;


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
      this.yoTime = yoTime;

      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightAboveGroundTrajectoryDuration.set(0.5);
      offsetHeightAboveGround.set(defaultOffsetHeightAboveGround);
      offsetHeightAboveGroundTrajectoryOutput.set(defaultOffsetHeightAboveGround);
      offsetHeightAboveGround.addVariableChangedListener((v) -> goToDesiredOffsetOverTime(offsetHeightAboveGround.getDoubleValue(),
                                                                                          offsetHeightAboveGroundTrajectoryDuration.getDoubleValue(),
                                                                                          false));

      setMinimumLegLengthToGround(minimumHeightAboveGround);
      setNominalLegLengthToGround(nominalHeightAboveGround);
      setMaximumLegLengthToGround(maximumHeightAboveGround);

      heightWaypoints = new RecyclingArrayList<>(5, SupplierBuilder.indexedSupplier(this::createHeightWaypoint));

      splinedHeightTrajectory = new SplinedHeightTrajectory(registry, yoGraphicsListRegistry);

      setSupportLeg(RobotSide.LEFT);

      this.nominalDoubleSupportPercentageIn.set(doubleSupportPercentageIn);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry == null)
         visualize = false;

      if (visualize)
      {
         double pointSize = 0.03;

         String prefix = "better_";

         List<AppearanceDefinition> colors = new ArrayList<>();
         colors.add(YoAppearance.CadetBlue());
         colors.add(YoAppearance.Chartreuse());
         colors.add(YoAppearance.Yellow());
         colors.add(YoAppearance.BlueViolet());
         colors.add(YoAppearance.Azure());

         String graphicListName = "BetterCoMHeightTrajectoryGenerator";

         heightWaypoints.clear();
         for (int i = 0; i < colors.size(); i++)
         {
            heightWaypoints.add().setupViz(graphicListName, prefix + "HeightWaypoint" + i, colors.get(i), yoGraphicsListRegistry);
         }

         YoGraphicPosition desiredCoMPositionViz = new YoGraphicPosition(prefix + "desiredCoMPosition",
                                                                         desiredCoMPosition,
                                                                         1.1 * pointSize,
                                                                         YoAppearance.Gold());

         yoGraphicsListRegistry.registerYoGraphic(graphicListName, desiredCoMPositionViz);
      }
      heightWaypoints.clear();
   }

   private CoMHeightTrajectoryWaypoint createHeightWaypoint(int i)
   {
      return new CoMHeightTrajectoryWaypoint("heightWaypoint" + i, registry);
   }

   public void reset()
   {
      tempFramePoint.setToZero(centerOfMassFrame);
      tempFramePoint.changeFrame(worldFrame);

      desiredCoMPosition.set(tempFramePoint);

      offsetHeightAboveGround.set(0.0, false);
      goToDesiredOffset(0.0);
   }

   public void setMinimumLegLengthToGround(double minimumHeightAboveGround)
   {
      this.minimumHeightAboveGround.set(minimumHeightAboveGround);
   }

   public void setNominalLegLengthToGround(double nominalHeightAboveGround)
   {
      this.nominalHeightAboveGround.set(nominalHeightAboveGround);
   }

   public void setMaximumLegLengthToGround(double maximumHeightAboveGround)
   {
      this.maximumHeightAboveGround.set(maximumHeightAboveGround);
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      frameOfLastFootstep = soleFrames.get(supportLeg);
      splinedHeightTrajectory.setReferenceFrame(frameOfLastFootstep);
   }

   public void initialize(NewTransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight, boolean isInTransfer)
   {
      FramePoint3DReadOnly transferToFootstepPosition = transferToAndNextFootstepsData.getTransferToPosition();

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

      tempFramePoint.setIncludingFrame(transferToAndNextFootstepsData.getCoMAtEndOfState());
      tempFramePoint.changeFrame(frameOfLastFootstep);
      doubleSupportPercentageIn.set(EuclidGeometryTools.percentageAlongLineSegment3D(startCoMPosition, middleCoMPosition, tempFramePoint));

      double midstanceWidth = 0.5 * middleCoMPosition.getY();

      startCoMPosition.setY(midstanceWidth);
      middleCoMPosition.setY(midstanceWidth);

      heightWaypoints.clear();

      computeWaypoints(startCoMPosition,
                       middleCoMPosition,
                       0.0,
                       middleAnkleZ,
                       minimumHeightAboveGround.getDoubleValue(),
                       maximumHeightAboveGround.getDoubleValue(),
                       isInTransfer);

      for (int i = 0; i < heightWaypoints.size(); i++)
      {
         heightWaypoints.get(i).setHeight(heightWaypoints.get(i).getHeight() + offsetHeightAboveGround.getDoubleValue());
         heightWaypoints.get(i).update();
      }

      splinedHeightTrajectory.clearWaypoints();
      splinedHeightTrajectory.addWaypoints(heightWaypoints);
      splinedHeightTrajectory.computeSpline();
   }

   private void computeWaypoints(FramePoint3DReadOnly startCoMPosition,
                                 FramePoint3DReadOnly endCoMPosition,
                                 double startGroundHeight,
                                 double endGroundHeight,
                                 double minimumHeight,
                                 double maximumHeight,
                                 boolean isInTransfer)
   {
      double midstanceWidth = 0.5 * (startCoMPosition.getY() + endCoMPosition.getY());

      double startX = startCoMPosition.getX();
      double endX = endCoMPosition.getX();

      double percentIn = MathTools.clamp(doubleSupportPercentageIn.getDoubleValue(), 0.0, 1.0);
      percentIn = percentIn > 0.5 ? 1.0 - percentIn : percentIn;
      percentIn = Math.max(percentIn, nominalDoubleSupportPercentageIn.getDoubleValue());

      double firstAlpha, secondAlpha, thirdAlpha;
      if (isInTransfer)
      {
         firstAlpha = 0.25;
         secondAlpha = 0.5;
         thirdAlpha = 0.75;
      }
      else
      {
         firstAlpha = 0.5 * percentIn;
         secondAlpha = percentIn;
         thirdAlpha = 0.5 * (1.0 + percentIn);
      }

      double firstMidpointX = InterpolationTools.linearInterpolate(startX, endX, firstAlpha);
      double secondMidpointX = InterpolationTools.linearInterpolate(startX, endX, secondAlpha);
      double thirdMidpointX = InterpolationTools.linearInterpolate(startX, endX, thirdAlpha);

      CoMHeightTrajectoryWaypoint startWaypoint = heightWaypoints.size() > 0 ? heightWaypoints.getLast() : getWaypointInFrame(frameOfLastFootstep);
      CoMHeightTrajectoryWaypoint firstMidpoint = getWaypointInFrame(frameOfLastFootstep);
      CoMHeightTrajectoryWaypoint secondMidpoint = getWaypointInFrame(frameOfLastFootstep);
      CoMHeightTrajectoryWaypoint thirdMidpoint = getWaypointInFrame(frameOfLastFootstep);
      CoMHeightTrajectoryWaypoint endWaypoint = getWaypointInFrame(frameOfLastFootstep);

      startWaypoint.setXY(startX, midstanceWidth);
      firstMidpoint.setXY(firstMidpointX, midstanceWidth);
      secondMidpoint.setXY(secondMidpointX, midstanceWidth);
      thirdMidpoint.setXY(thirdMidpointX, midstanceWidth);
      endWaypoint.setXY(endX, midstanceWidth);


      double startMinHeight = findWaypointHeight(minimumHeight, 0.0, startX, startGroundHeight);
      double startMaxHeight = findWaypointHeight(maximumHeight, 0.0, startX, startGroundHeight);
      double firstMinHeight = findWaypointHeight(minimumHeight, 0.0, firstMidpointX, startGroundHeight);
      double firstMaxHeight = findWaypointHeight(maximumHeight, 0.0, firstMidpointX, startGroundHeight);
      double secondMinHeight = Math.max(findWaypointHeight(minimumHeight, 0.0, secondMidpointX, startGroundHeight),
                                        findWaypointHeight(minimumHeight, endX, secondMidpointX, endGroundHeight));
      double secondMaxHeight = Math.min(findWaypointHeight(maximumHeight, 0.0, secondMidpointX, startGroundHeight),
                                        findWaypointHeight(maximumHeight, endX, secondMidpointX, endGroundHeight));
      double thirdMinHeight = findWaypointHeight(minimumHeight, endX, thirdMidpointX, endGroundHeight);
      double thirdMaxHeight = findWaypointHeight(maximumHeight, endX, thirdMidpointX, endGroundHeight);
      double endMinHeight = minimumHeight + endGroundHeight;
      double endMaxHeight = maximumHeight + endGroundHeight;

      startWaypoint.setMinMax(startMinHeight, startMaxHeight);
      firstMidpoint.setMinMax(firstMinHeight, firstMaxHeight);
      secondMidpoint.setMinMax(secondMinHeight, secondMaxHeight);
      thirdMidpoint.setMinMax(thirdMinHeight, thirdMaxHeight);
      endWaypoint.setMinMax(endMinHeight, endMaxHeight);

      startWaypoint.setHeight(MathTools.clamp(startCoMPosition.getZ(), startMinHeight, startMaxHeight));
      endWaypoint.setHeight(MathTools.clamp(endCoMPosition.getZ() + offsetHeightAboveGround.getDoubleValue(), endMinHeight, endMaxHeight));
   }

   private CoMHeightTrajectoryWaypoint getWaypointInFrame(ReferenceFrame referenceFrame)
   {
      CoMHeightTrajectoryWaypoint waypoint = heightWaypoints.add();
      waypoint.setToZero(referenceFrame);

      return waypoint;
   }

   private static double findWaypointHeight(double desiredDistanceFromFoot, double startAnkleX, double queryX, double extraHeight)
   {
      return Math.sqrt(MathTools.square(desiredDistanceFromFoot) - MathTools.square(queryX - startAnkleX)) + extraHeight;
   }

   public void solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, boolean isInDoubleSupport)
   {
      com.setToZero(centerOfMassFrame);
      com.changeFrame(worldFrame);

      solve(comHeightPartialDerivativesDataToPack, com, isInDoubleSupport);

      desiredCoMPosition.set(com.getX(), com.getY(), comHeightPartialDerivativesDataToPack.getComHeight());
   }

   private final Point2D point = new Point2D();

   private void solve(CoMHeightPartialDerivativesData comHeightPartialDerivativesDataToPack, FramePoint3DBasics queryPoint, boolean isInDoubleSupport)
   {
      this.queryPosition.set(queryPoint);

      handleInitializeToCurrent();
      updateTrajectoryOffsets();

      offsetHeightAboveGroundTrajectoryOutput.set(offsetHeightTrajectoryGenerator.getValue());

      percentageThroughSegment.set(splinedHeightTrajectory.solve(comHeightPartialDerivativesDataToPack, queryPoint, point));
      point.addY(offsetHeightAboveGround.getDoubleValue());
      comHeightPartialDerivativesDataToPack.setCoMHeight(worldFrame,
                                                         comHeightPartialDerivativesDataToPack.getComHeight() + offsetHeightAboveGround.getDoubleValue());

      this.splineQuery.set(point.getX());
      this.desiredCoMHeight.set(point.getY());
   }

   private void handleInitializeToCurrent()
   {
      if (!initializeToCurrent.getBooleanValue())
         return;

      initializeToCurrent.set(false);

      tempFramePoint.setToZero(centerOfMassFrame);
      tempFramePoint.changeFrame(frameOfLastFootstep);

      double heightOffset = tempFramePoint.getZ() - desiredCoMHeight.getDoubleValue();

      offsetHeightAboveGround.set(heightOffset);
      goToDesiredOffset(offsetHeightAboveGround.getDoubleValue());
   }

   private void updateTrajectoryOffsets()
   {
      if (isTrajectoryOffsetStopped.getBooleanValue())
         return;

      double deltaTime = yoTime.getValue() - offsetHeightAboveGroundChangedTime.getDoubleValue();

      if (!offsetHeightTrajectoryGenerator.isEmpty())
         offsetHeightTrajectoryGenerator.compute(deltaTime);

      if (consumeHeightOffsetCommands())
         offsetHeightTrajectoryGenerator.compute(deltaTime);

      offsetHeightAboveGround.set(offsetHeightTrajectoryGenerator.getValue(), false);
   }

   private boolean consumeHeightOffsetCommands()
   {
      if (offsetHeightTrajectoryGenerator.isDone() || commandQueue.isEmpty())
         return false;

      double firstTrajectoryPointTime = offsetHeightTrajectoryGenerator.getLastWaypointTime();
      PelvisHeightTrajectoryCommand command = commandQueue.poll();
      numberOfQueuedCommands.decrement();

      command.getEuclideanTrajectory().addTimeOffset(firstTrajectoryPointTime);

      offsetHeightTrajectoryGenerator.clear();

      // add the current desired height offset as the start
      if (command.getEuclideanTrajectory().getTrajectoryPoint(0).getTime() > firstTrajectoryPointTime + 1.0e-5)
         offsetHeightTrajectoryGenerator.appendWaypoint(0.0, offsetHeightAboveGroundTrajectoryOutput.getDoubleValue(), 0.0);

      int numberOfTrajectoryPoints = queueExceedingTrajectoryPointsIfNeeded(command);

      for (int trajectoryPointIndex = 0; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
         appendTrajectoryPoint(command.getEuclideanTrajectory().getTrajectoryPoint(trajectoryPointIndex));

      offsetHeightTrajectoryGenerator.initialize();
      isTrajectoryOffsetStopped.set(false);
      return true;
   }

   private int queueExceedingTrajectoryPointsIfNeeded(PelvisHeightTrajectoryCommand command)
   {
      int numberOfTrajectoryPoints = command.getEuclideanTrajectory().getNumberOfTrajectoryPoints();

      int maximumNumberOfWaypoints =
            offsetHeightTrajectoryGenerator.getMaximumNumberOfWaypoints() - offsetHeightTrajectoryGenerator.getCurrentNumberOfWaypoints();

      if (numberOfTrajectoryPoints <= maximumNumberOfWaypoints)
         return numberOfTrajectoryPoints;

      PelvisHeightTrajectoryCommand commandForExcedent = commandQueue.addFirst();
      numberOfQueuedCommands.increment();
      commandForExcedent.clear();
      commandForExcedent.getEuclideanTrajectory().setPropertiesOnly(command.getEuclideanTrajectory());

      for (int trajectoryPointIndex = maximumNumberOfWaypoints; trajectoryPointIndex < numberOfTrajectoryPoints; trajectoryPointIndex++)
      {
         commandForExcedent.getEuclideanTrajectory().addTrajectoryPoint(command.getEuclideanTrajectory().getTrajectoryPoint(trajectoryPointIndex));
      }

      double timeOffsetToSubtract = command.getEuclideanTrajectory().getTrajectoryPoint(maximumNumberOfWaypoints - 1).getTime();
      commandForExcedent.getEuclideanTrajectory().subtractTimeOffset(timeOffsetToSubtract);

      return maximumNumberOfWaypoints;
   }

   private void appendTrajectoryPoint(FrameEuclideanTrajectoryPoint trajectoryPoint)
   {
      appendTrajectoryPoint(trajectoryPoint.getReferenceFrame(),
                            trajectoryPoint.getPositionZ(),
                            trajectoryPoint.getLinearVelocityZ(),
                            trajectoryPoint.getTime());
   }

   private void appendTrajectoryPoint(ReferenceFrame frameOfPoint, double desiredHeight, double desiredVelocity, double pointTime)
   {
      tempFramePoint.setIncludingFrame(frameOfPoint, 0.0, 0.0, desiredHeight);
      tempFramePoint.changeFrame(frameOfLastFootstep);

      double zOffset = tempFramePoint.getZ() - desiredCoMHeight.getDoubleValue();
      offsetHeightTrajectoryGenerator.appendWaypoint(pointTime, zOffset, desiredVelocity);
   }

   public void goHome(double trajectoryTime)
   {
      if (!processGoHome.getValue())
         return;

      goToDesiredOffsetOverTime(0.0, trajectoryTime, true);
   }

   public void goToDesiredOffsetOverTime(double desiredOffset, double durationToMove, boolean updateMoving)
   {
      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightTrajectoryGenerator.clear();
      offsetHeightTrajectoryGenerator.appendWaypoint(0.0, offsetHeightAboveGroundTrajectoryOutput.getDoubleValue(), 0.0);
      offsetHeightTrajectoryGenerator.appendWaypoint(durationToMove, desiredOffset, 0.0);
      offsetHeightTrajectoryGenerator.initialize();

      if (updateMoving)
         isTrajectoryOffsetStopped.set(false);
   }

   public void goToDesiredOffset(double desiredOffset)
   {
      offsetHeightAboveGroundChangedTime.set(yoTime.getValue());
      offsetHeightTrajectoryGenerator.clear();
      offsetHeightTrajectoryGenerator.appendWaypoint(0.0, desiredOffset, 0.0);
      offsetHeightTrajectoryGenerator.initialize();
      isTrajectoryOffsetStopped.set(false);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      isTrajectoryOffsetStopped.set(command.isStopAllTrajectory());
      offsetHeightAboveGround.set(offsetHeightAboveGroundTrajectoryOutput.getDoubleValue());
   }

   public void initializeDesiredHeightToCurrent()
   {
      initializeToCurrent.set(true);
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
