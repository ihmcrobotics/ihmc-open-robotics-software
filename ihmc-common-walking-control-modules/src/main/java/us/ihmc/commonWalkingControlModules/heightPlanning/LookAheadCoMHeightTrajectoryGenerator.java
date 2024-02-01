package us.ihmc.commonWalkingControlModules.heightPlanning;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisHeightTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PelvisTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class LookAheadCoMHeightTrajectoryGenerator implements SCS2YoGraphicHolder
{
   private static final double defaultPercentageInOffset = 0.05;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private boolean visualize = true;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean initializeToCurrent = new YoBoolean("initializeCoMHeightToCurrent", registry);
   private final BooleanProvider processGoHome = new BooleanParameter("ProcessGoHome", registry, false);

   private final HeightOffsetHandler heightOffsetHandler;
   private final YoEnum<RobotSide> supportLegSide = new YoEnum<>("supportLegSide", registry, RobotSide.class);

   private final YoDouble minimumLegLength = new YoDouble("minimumLegLength", registry);
   private final YoDouble nominalLegLength = new YoDouble("nominalLegLength", registry);
   private final YoDouble maximumLegLength = new YoDouble("maximumLegLength", registry);
   private final YoDouble heightChangeForNonFlatStep = new YoDouble("heightChangeForNonFlatStep", registry);
   private final YoDouble maxLengthReductionSteppingDown = new YoDouble("maxLengthReductionSteppingDown", registry);
   private final YoDouble hipWidth = new YoDouble("hipWidth", registry);
   private final YoDouble extraToeOffHeight = new YoDouble("extraToeOffHeight", registry);

   private final YoDouble nominalDoubleSupportExchange = new YoDouble("nominalDoubleSupportExchange", registry);
   private final YoDouble doubleSupportExchange = new YoDouble("doubleSupportExchange", registry);
   private final YoDouble doubleSupportPercentageIn = new YoDouble("doubleSupportPercentageIn", registry);
   private final YoDouble doubleSupportPercentageOut = new YoDouble("doubleSupportPercentageOut", registry);
   private final YoDouble doubleSupportExchangeOffset = new YoDouble("doubleSupportExchangeOffset", registry);
   private final YoDouble percentageThroughSegment = new YoDouble("percentageThroughSegment", registry);
   private final YoDouble splineQuery = new YoDouble("splineQuery", registry);

   private final FramePoint3D transferFromPosition = new FramePoint3D();
   private final FramePoint3D transferToPosition = new FramePoint3D();
   private final YoFramePoint3D yoTransferFromPosition = new YoFramePoint3D("transferFromPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D yoTransferToPosition = new YoFramePoint3D("transferToPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D desiredCoMPositionAtStart = new YoFramePoint3D("desiredCoMPositionAtStart", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2D desiredCoMPositionAtEnd = new YoFramePoint2D("desiredCoMPositionAtEnd", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble desiredCoMHeight = new YoDouble("desiredCoMHeight", registry);
   private final YoDouble desiredCoMSlope = new YoDouble("desiredCoMSlope", registry);
   private final YoDouble desiredCoMSlopeAtStart = new YoDouble("desiredCoMSlopeAtStart", registry);
   private final YoFramePoint3D queryPosition = new YoFramePoint3D("queryPosition", ReferenceFrame.getWorldFrame(), registry);

   private final RecyclingArrayList<CoMHeightTrajectoryWaypoint> heightWaypoints;

   private final DoubleProvider yoTime;

   private ReferenceFrame frameOfSupportLeg;
   private final ReferenceFrame centerOfMassFrame;
   private final ReferenceFrame frameOfHeight;
   private final SideDependentList<? extends ReferenceFrame> soleFrames;

   private final FixedFramePoint3DBasics com = new FramePoint3D();

   private final FramePoint3D tempFramePoint = new FramePoint3D();

   private final FramePoint3D startCoMPosition = new FramePoint3D();
   private final FramePoint3D endCoMPosition = new FramePoint3D();

   private final SplinedHeightTrajectory splinedHeightTrajectory;

   public LookAheadCoMHeightTrajectoryGenerator(double minimumLegLength,
                                                double nominalLegLength,
                                                double maximumLegLength,
                                                double nominalDoubleSupportExchange,
                                                double heightThresholdForNonFlat,
                                                double maxLengthReductionSteppingDown,
                                                ReferenceFrame centerOfMassFrame,
                                                ReferenceFrame frameOfHeight,
                                                SideDependentList<? extends ReferenceFrame> soleFrames,
                                                DoubleProvider yoTime,
                                                YoGraphicsListRegistry yoGraphicsListRegistry,
                                                YoRegistry parentRegistry)
   {
      this(minimumLegLength,
           nominalLegLength,
           maximumLegLength,
           nominalDoubleSupportExchange,
           heightThresholdForNonFlat,
           maxLengthReductionSteppingDown,
           0.0,
           centerOfMassFrame,
           frameOfHeight,
           soleFrames,
           yoTime,
           yoGraphicsListRegistry,
           parentRegistry);
   }

   public LookAheadCoMHeightTrajectoryGenerator(double minimumLegLength,
                                                double nominalLegLength,
                                                double maximumLegLength,
                                                double nominalDoubleSupportExchange,
                                                double heightChangeForNonFlatStep,
                                                double maxLengthReductionSteppingDown,
                                                double hipWidth,
                                                ReferenceFrame centerOfMassFrame,
                                                ReferenceFrame frameOfHeight,
                                                SideDependentList<? extends ReferenceFrame> soleFrames,
                                                DoubleProvider yoTime,
                                                YoGraphicsListRegistry yoGraphicsListRegistry,
                                                YoRegistry parentRegistry)
   {
      this.centerOfMassFrame = centerOfMassFrame;
      this.frameOfHeight = frameOfHeight;
      this.soleFrames = soleFrames;
      this.yoTime = yoTime;

      heightOffsetHandler = new HeightOffsetHandler(yoTime, registry);
      doubleSupportExchangeOffset.set(defaultPercentageInOffset);

      this.minimumLegLength.set(minimumLegLength);
      this.nominalLegLength.set(nominalLegLength);
      this.maximumLegLength.set(maximumLegLength);
      this.heightChangeForNonFlatStep.set(heightChangeForNonFlatStep);
      this.maxLengthReductionSteppingDown.set(maxLengthReductionSteppingDown);
      this.hipWidth.set(hipWidth);

      heightWaypoints = new RecyclingArrayList<>(6, SupplierBuilder.indexedSupplier(this::createHeightWaypoint));

      splinedHeightTrajectory = new SplinedHeightTrajectory(registry, yoGraphicsListRegistry);

      setSupportLeg(RobotSide.LEFT);

      this.nominalDoubleSupportExchange.set(nominalDoubleSupportExchange);

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
         colors.add(YoAppearance.Yellow());
         colors.add(YoAppearance.BlueViolet());
         colors.add(YoAppearance.Azure());

         String graphicListName = "CoMHeightTrajectoryGenerator";

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
      tempFramePoint.setToZero(frameOfHeight);
      tempFramePoint.changeFrame(frameOfSupportLeg);
      tempFramePoint.setZ(nominalLegLength.getDoubleValue());

      desiredCoMHeight.set(nominalLegLength.getDoubleValue());
      desiredCoMPosition.setMatchingFrame(tempFramePoint);
      desiredCoMSlope.set(0.0);

      extraToeOffHeight.set(0.0);

      heightOffsetHandler.reset();
   }

   public double getDoubleSupportPercentageIn()
   {
      return doubleSupportPercentageIn.getDoubleValue();
   }

   public void setSupportLeg(RobotSide supportLeg)
   {
      supportLegSide.set(supportLeg);
      frameOfSupportLeg = soleFrames.get(supportLeg);
      splinedHeightTrajectory.setReferenceFrame(frameOfSupportLeg);
      heightOffsetHandler.setReferenceFrame(frameOfSupportLeg);
   }

   public void initializeToNominalHeight()
   {
      heightWaypoints.clear();
      desiredCoMSlope.set(0.0);

      if (soleFrames.get(RobotSide.LEFT).getTransformToRoot().getTranslationZ() > soleFrames.get(RobotSide.RIGHT).getTransformToRoot().getTranslationZ())
         setSupportLeg(RobotSide.RIGHT);
      else
         setSupportLeg(RobotSide.LEFT);

      transferToPosition.setToZero(soleFrames.get(RobotSide.LEFT));
      transferFromPosition.setToZero(soleFrames.get(RobotSide.RIGHT));

      transferToPosition.changeFrame(frameOfSupportLeg);
      transferFromPosition.changeFrame(frameOfSupportLeg);

      yoTransferFromPosition.setMatchingFrame(transferFromPosition);
      yoTransferToPosition.setMatchingFrame(transferToPosition);

      tempFramePoint.setToZero(frameOfSupportLeg);
      tempFramePoint.setZ(nominalLegLength.getDoubleValue());
      tempFramePoint.changeFrame(worldFrame);

      double midstanceY = 0.5 * (transferToPosition.getY() + transferFromPosition.getY());

      CoMHeightTrajectoryWaypoint startWaypoint = getWaypointInFrame(frameOfSupportLeg);
      startWaypoint.setHeight(nominalLegLength.getDoubleValue());
      startWaypoint.setMinMax(nominalLegLength.getDoubleValue() - 0.05, nominalLegLength.getDoubleValue() + 0.05);
      startWaypoint.setXY(transferFromPosition.getX(), midstanceY);

      CoMHeightTrajectoryWaypoint endWaypoint = getWaypointInFrame(frameOfSupportLeg);
      endWaypoint.setHeight(nominalLegLength.getDoubleValue());
      endWaypoint.setMinMax(nominalLegLength.getDoubleValue() - 0.05, nominalLegLength.getDoubleValue() + 0.05);
      endWaypoint.setXY(transferToPosition.getX(), midstanceY);

      // set the x and y value to the current x and y, so that we get a continuous trajectory.
      com.setFromReferenceFrame(centerOfMassFrame);
      desiredCoMPosition.setX(com.getX());
      desiredCoMPosition.setY(com.getY());

      // store the desired value
      desiredCoMPositionAtStart.set(desiredCoMPosition);
      desiredCoMPositionAtStart.setZ(tempFramePoint.getZ());
      desiredCoMSlopeAtStart.set(desiredCoMSlope.getDoubleValue());

      for (int i = 0; i < heightWaypoints.size(); i++)
         heightWaypoints.get(i).update();

      splinedHeightTrajectory.clearWaypoints();
      splinedHeightTrajectory.addWaypoints(heightWaypoints);
      splinedHeightTrajectory.computeSpline(desiredCoMSlopeAtStart.getDoubleValue());
   }

   public void initialize(TransferToAndNextFootstepsData transferToAndNextFootstepsData, double extraToeOffHeight)
   {
      FramePoint3DReadOnly transferToFootstepPosition = transferToAndNextFootstepsData.getTransferToPosition();
      this.extraToeOffHeight.set(extraToeOffHeight);

      transferToPosition.setIncludingFrame(transferToFootstepPosition);
      transferFromPosition.setToZero(soleFrames.get(transferToAndNextFootstepsData.getTransferToSide().getOppositeSide()));
      transferToPosition.changeFrame(frameOfSupportLeg);
      transferFromPosition.changeFrame(frameOfSupportLeg);
      double startFootZ = transferFromPosition.getZ();

      yoTransferFromPosition.setMatchingFrame(transferFromPosition);
      yoTransferToPosition.setMatchingFrame(transferToPosition);

      // set the x and y value to the current x and y, so that we get a continuous trajectory.
      com.setFromReferenceFrame(centerOfMassFrame);
      desiredCoMPosition.setX(com.getX());
      desiredCoMPosition.setY(com.getY());

      // store the desired value
      desiredCoMPositionAtStart.set(desiredCoMPosition);
      desiredCoMSlopeAtStart.set(desiredCoMSlope.getDoubleValue());

      startCoMPosition.setIncludingFrame(desiredCoMPosition);
      startCoMPosition.changeFrame(frameOfSupportLeg);

      endCoMPosition.setIncludingFrame(transferToFootstepPosition);
      endCoMPosition.changeFrame(frameOfSupportLeg);
      double endFootZ = endCoMPosition.getZ();
      endCoMPosition.addZ(nominalLegLength.getDoubleValue());

      double midstanceY = 0.5 * (transferToPosition.getY() + transferFromPosition.getY());
      startCoMPosition.setY(midstanceY);
      endCoMPosition.setY(midstanceY);

      desiredCoMPositionAtEnd.set(transferToAndNextFootstepsData.getCoMAtEndOfState());

      if (!desiredCoMPositionAtEnd.containsNaN())
      {
         tempFramePoint.setIncludingFrame(transferToAndNextFootstepsData.getCoMAtEndOfState(), 0.0);
         tempFramePoint.changeFrame(frameOfSupportLeg);
         tempFramePoint.setZ(nominalLegLength.getDoubleValue());

         double percentExchange = EuclidGeometryTools.percentageAlongLineSegment3D(tempFramePoint, startCoMPosition, endCoMPosition);

         percentExchange = MathTools.clamp(percentExchange, nominalDoubleSupportExchange.getDoubleValue(), 1.0 - nominalDoubleSupportExchange.getDoubleValue());
         doubleSupportExchange.set(percentExchange);
      }
      else
      {
         doubleSupportExchange.set(nominalDoubleSupportExchange.getDoubleValue());
      }

      doubleSupportPercentageIn.set(doubleSupportExchange.getDoubleValue() - doubleSupportExchangeOffset.getDoubleValue());
      doubleSupportPercentageOut.set(doubleSupportExchange.getDoubleValue() + doubleSupportExchangeOffset.getDoubleValue());

      heightWaypoints.clear();

      computeWaypoints(startCoMPosition,
                       endCoMPosition,
                       startFootZ,
                       endFootZ,
                       minimumLegLength.getDoubleValue(),
                       maximumLegLength.getDoubleValue(),
                       this.extraToeOffHeight.getDoubleValue());

      for (int i = 0; i < heightWaypoints.size(); i++)
         heightWaypoints.get(i).update();

      splinedHeightTrajectory.clearWaypoints();
      splinedHeightTrajectory.addWaypoints(heightWaypoints);
      splinedHeightTrajectory.computeSpline(desiredCoMSlopeAtStart.getDoubleValue());
   }

   // FIXME this doesn't work well during transfer.
   private void computeWaypoints(FramePoint3DReadOnly startCoMPosition,
                                 FramePoint3DReadOnly endCoMPosition,
                                 double startGroundHeight,
                                 double endGroundHeight,
                                 double minimumLength,
                                 double maximumLength,
                                 double extraToeOffHeight)
   {
      double startAnkleX = transferFromPosition.getX();
      double startAnkleY = transferFromPosition.getY();
      double endAnkleX = transferToPosition.getX();
      double endAnkleY = transferToPosition.getY();

      double stepHeightChange = transferToPosition.getZ() - transferFromPosition.getZ();
      boolean isSteppingDown = stepHeightChange < -heightChangeForNonFlatStep.getValue();
      double heightReductionDuringExchange = isSteppingDown ? maxLengthReductionSteppingDown.getDoubleValue() : 0.0;

      double startWaypointX = startCoMPosition.getX();
      double endWaypointX = endCoMPosition.getX();
      double startWaypointY = startCoMPosition.getY();
      double endWaypointY = endCoMPosition.getY();

      double firstAlpha = 0.5 * doubleSupportPercentageIn.getDoubleValue();
      double secondAlpha = doubleSupportPercentageIn.getDoubleValue();
      double thirdAlpha = doubleSupportPercentageOut.getDoubleValue();
      double fourthAlpha = 0.5 * (1.0 + doubleSupportPercentageOut.getDoubleValue());

      double firstMidpointX = InterpolationTools.linearInterpolate(startWaypointX, endWaypointX, firstAlpha);
      double secondMidpointX = InterpolationTools.linearInterpolate(startWaypointX, endWaypointX, secondAlpha);
      double thirdMidpointX = InterpolationTools.linearInterpolate(startWaypointX, endWaypointX, thirdAlpha);
      double fourthMidpointX = InterpolationTools.linearInterpolate(startWaypointX, endWaypointX, fourthAlpha);

      double firstMidpointY = InterpolationTools.linearInterpolate(startWaypointY, endWaypointY, firstAlpha);
      double secondMidpointY = InterpolationTools.linearInterpolate(startWaypointY, endWaypointY, secondAlpha);
      double thirdMidpointY = InterpolationTools.linearInterpolate(startWaypointY, endWaypointY, thirdAlpha);
      double fourthMidpointY = InterpolationTools.linearInterpolate(startWaypointY, endWaypointY, fourthAlpha);

      CoMHeightTrajectoryWaypoint startWaypoint = !heightWaypoints.isEmpty() ? heightWaypoints.getLast() : getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint firstMidpoint = getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint secondMidpoint = getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint thirdMidpoint = getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint fourthMidpoint = getWaypointInFrame(frameOfSupportLeg);
      CoMHeightTrajectoryWaypoint endWaypoint = getWaypointInFrame(frameOfSupportLeg);

      startWaypoint.setXY(startWaypointX, startWaypointY);
      firstMidpoint.setXY(firstMidpointX, firstMidpointY);
      secondMidpoint.setXY(secondMidpointX, secondMidpointY);
      thirdMidpoint.setXY(thirdMidpointX, thirdMidpointY);
      fourthMidpoint.setXY(fourthMidpointX, fourthMidpointY);
      endWaypoint.setXY(endWaypointX, endWaypointY);

      double startMinHeight = findWaypointHeight(minimumLength,
                                                 hipWidth.getDoubleValue(),
                                                 startAnkleX,
                                                 startAnkleY,
                                                 startWaypointX,
                                                 startWaypointY,
                                                 startGroundHeight);
      double startMaxHeight = findWaypointHeight(maximumLength + extraToeOffHeight,
                                                 hipWidth.getDoubleValue(),
                                                 startAnkleX,
                                                 startAnkleY,
                                                 startWaypointX,
                                                 startWaypointY,
                                                 startGroundHeight);
      startMinHeight = Math.min(startMinHeight, startCoMPosition.getZ() - heightOffsetHandler.getOffsetHeightAboveGround());
      startMaxHeight = Math.max(startMaxHeight, startCoMPosition.getZ() - heightOffsetHandler.getOffsetHeightAboveGround());
      double firstMinHeight = findWaypointHeight(minimumLength,
                                                 hipWidth.getDoubleValue(),
                                                 startAnkleX,
                                                 startAnkleY,
                                                 firstMidpointX,
                                                 firstMidpointY,
                                                 startGroundHeight);
      double firstMaxHeight = findWaypointHeight(maximumLength,
                                                 hipWidth.getDoubleValue(),
                                                 startAnkleX,
                                                 startAnkleY,
                                                 firstMidpointX,
                                                 firstMidpointY,
                                                 startGroundHeight);
      firstMinHeight = Math.min(firstMinHeight, startMinHeight);
      firstMaxHeight = Math.min(firstMaxHeight, startMaxHeight);

      double exchangeInFromMinHeight = findWaypointHeight(minimumLength,
                                                          hipWidth.getDoubleValue(),
                                                          startAnkleX,
                                                          startAnkleY,
                                                          secondMidpointX,
                                                          secondMidpointY,
                                                          startGroundHeight);
      double exchangeInToMinHeight = findWaypointHeight(minimumLength,
                                                        hipWidth.getDoubleValue(),
                                                        endAnkleX,
                                                        endAnkleY,
                                                        secondMidpointX,
                                                        secondMidpointY,
                                                        endGroundHeight);
      double exchangeInFromMaxHeight = findWaypointHeight(maximumLength + extraToeOffHeight - heightReductionDuringExchange,
                                                          hipWidth.getDoubleValue(),
                                                          startAnkleX,
                                                          startAnkleY,
                                                          secondMidpointX,
                                                          secondMidpointY,
                                                          startGroundHeight);
      double exchangeInToMaxHeight = findWaypointHeight(maximumLength - heightReductionDuringExchange,
                                                        hipWidth.getDoubleValue(),
                                                        endAnkleX,
                                                        endAnkleY,
                                                        secondMidpointX,
                                                        secondMidpointY,
                                                        endGroundHeight);
      double secondMinHeight = Math.min(Math.max(exchangeInFromMinHeight, exchangeInToMinHeight), Math.min(exchangeInFromMaxHeight, exchangeInToMaxHeight));
      double secondMaxHeight = Math.min(exchangeInFromMaxHeight, exchangeInToMaxHeight);

      double exchangeOutFromMinHeight = findWaypointHeight(minimumLength,
                                                           hipWidth.getDoubleValue(),
                                                           startAnkleX,
                                                           startAnkleY,
                                                           thirdMidpointX,
                                                           thirdMidpointY,
                                                           startGroundHeight);
      double exchangeOutToMinHeight = findWaypointHeight(minimumLength,
                                                         hipWidth.getDoubleValue(),
                                                         endAnkleX,
                                                         endAnkleY,
                                                         thirdMidpointX,
                                                         thirdMidpointY,
                                                         endGroundHeight);
      double exchangeOutFromMaxHeight = findWaypointHeight(maximumLength + extraToeOffHeight - heightReductionDuringExchange,
                                                           hipWidth.getDoubleValue(),
                                                           startAnkleX,
                                                           startAnkleY,
                                                           thirdMidpointX,
                                                           thirdMidpointY,
                                                           startGroundHeight);
      double exchangeOutToMaxHeight = findWaypointHeight(maximumLength - heightReductionDuringExchange,
                                                         hipWidth.getDoubleValue(),
                                                         endAnkleX,
                                                         endAnkleY,
                                                         thirdMidpointX,
                                                         thirdMidpointY,
                                                         endGroundHeight);
      double thirdMinHeight = Math.min(Math.max(exchangeOutFromMinHeight, exchangeOutToMinHeight), Math.min(exchangeOutFromMaxHeight, exchangeOutToMaxHeight));
      double thirdMaxHeight = Math.min(exchangeOutFromMaxHeight, exchangeOutToMaxHeight);

      double fourthMinHeight = findWaypointHeight(minimumLength,
                                                  hipWidth.getDoubleValue(),
                                                  endAnkleX,
                                                  endAnkleY,
                                                  fourthMidpointX,
                                                  fourthMidpointY,
                                                  endGroundHeight);
      double fourthMaxHeight = findWaypointHeight(maximumLength,
                                                  hipWidth.getDoubleValue(),
                                                  endAnkleX,
                                                  endAnkleY,
                                                  fourthMidpointX,
                                                  fourthMidpointY,
                                                  endGroundHeight);
      double endMinHeight = minimumLength + endGroundHeight;
      double endMaxHeight = maximumLength + endGroundHeight;

      fourthMinHeight = Math.min(fourthMinHeight, endMinHeight);
      fourthMaxHeight = Math.min(fourthMaxHeight, endMaxHeight);

      startWaypoint.setMinMax(startMinHeight, startMaxHeight);
      firstMidpoint.setMinMax(firstMinHeight, firstMaxHeight);
      secondMidpoint.setMinMax(secondMinHeight, secondMaxHeight);
      thirdMidpoint.setMinMax(thirdMinHeight, thirdMaxHeight);
      fourthMidpoint.setMinMax(fourthMinHeight, fourthMaxHeight);
      endWaypoint.setMinMax(endMinHeight, endMaxHeight);

      //      startWaypoint.setHeight(MathTools.clamp(startCoMPosition.getZ(), startMinHeight, startMaxHeight));
      startWaypoint.setHeight(startCoMPosition.getZ() - heightOffsetHandler.getOffsetHeightAboveGround());
      endWaypoint.setHeight(MathTools.clamp(endCoMPosition.getZ(), endMinHeight, endMaxHeight));
   }

   private CoMHeightTrajectoryWaypoint getWaypointInFrame(ReferenceFrame referenceFrame)
   {
      CoMHeightTrajectoryWaypoint waypoint = heightWaypoints.add();
      waypoint.setToZero(referenceFrame);

      return waypoint;
   }

   private static double findWaypointHeight(double desiredDistance,
                                            double hipWidth,
                                            double ankleX,
                                            double ankleY,
                                            double queryX,
                                            double queryY,
                                            double extraHeight)
   {
      double widthDisplacement = queryY - ankleY - Math.signum(queryY - ankleY) * 0.5 * hipWidth;
      double height = EuclidCoreTools.squareRoot(MathTools.square(desiredDistance) - MathTools.square(queryX - ankleX) - MathTools.square(widthDisplacement));
      return height + extraHeight;
   }

   public void solve(CoMHeightPartialDerivativesDataBasics comHeightPartialDerivativesDataToPack)
   {
      com.setFromReferenceFrame(centerOfMassFrame);

      solve(comHeightPartialDerivativesDataToPack, com);

      desiredCoMPosition.set(com.getX(), com.getY(), comHeightPartialDerivativesDataToPack.getComHeight());
   }

   private final Point2D point = new Point2D();

   void solve(CoMHeightPartialDerivativesDataBasics comHeightPartialDerivativesDataToPack, FixedFramePoint3DBasics queryPoint)
   {
      this.queryPosition.set(queryPoint);

      percentageThroughSegment.set(splinedHeightTrajectory.solve(comHeightPartialDerivativesDataToPack, queryPoint, point));

      heightOffsetHandler.update(splinedHeightTrajectory.getHeightSplineSetpoint());

      handleInitializeToCurrent(point.getY() + heightOffsetHandler.getOffsetHeightAboveGround());

      point.addY(heightOffsetHandler.getOffsetHeightAboveGround());
      comHeightPartialDerivativesDataToPack.setCoMHeight(worldFrame,
                                                         comHeightPartialDerivativesDataToPack.getComHeight()
                                                                     + heightOffsetHandler.getOffsetHeightAboveGround());

      this.splineQuery.set(point.getX());
      this.desiredCoMHeight.set(point.getY());
      desiredCoMSlope.set(splinedHeightTrajectory.getPartialDzDs());
   }

   private void handleInitializeToCurrent(double normalDesiredHeight)
   {
      if (!initializeToCurrent.getBooleanValue())
         return;

      initializeToCurrent.set(false);

      tempFramePoint.setToZero(frameOfHeight);
      tempFramePoint.changeFrame(frameOfSupportLeg);

      double heightOffset = tempFramePoint.getZ() - normalDesiredHeight + heightOffsetHandler.getOffsetHeightAboveGround();

      heightOffsetHandler.initializeToCurrent(heightOffset);
   }

   private final PelvisHeightTrajectoryCommand tempPelvisHeightTrajectoryCommand = new PelvisHeightTrajectoryCommand();

   public boolean handlePelvisTrajectoryCommand(PelvisTrajectoryCommand command)
   {
      SE3TrajectoryControllerCommand se3Trajectory = command.getSE3Trajectory();

      if (!se3Trajectory.getSelectionMatrix().isLinearZSelected())
         return false; // The user does not want to control the height, do nothing.

      se3Trajectory.changeFrame(worldFrame);
      tempPelvisHeightTrajectoryCommand.set(command);
      handlePelvisHeightTrajectoryCommand(tempPelvisHeightTrajectoryCommand);
      return true;
   }

   public boolean handlePelvisHeightTrajectoryCommand(PelvisHeightTrajectoryCommand command)
   {
      return heightOffsetHandler.handlePelvisHeightTrajectoryCommand(command, splinedHeightTrajectory.getHeightSplineSetpoint());
   }

   public void goHome(double trajectoryTime)
   {
      if (!processGoHome.getValue())
         return;

      heightOffsetHandler.goHome(trajectoryTime);
   }

   public void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command)
   {
      heightOffsetHandler.handleStopAllTrajectoryCommand(command);
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
      return yoTime.getValue() - heightOffsetHandler.getOffsetHeightAboveGroundChangedTime();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(splinedHeightTrajectory.getSCS2YoGraphics());

      if (visualize)
      {
         List<ColorDefinition> colors = new ArrayList<>();
         colors.add(ColorDefinitions.CadetBlue());
         colors.add(ColorDefinitions.Chartreuse());
         colors.add(ColorDefinitions.Yellow());
         colors.add(ColorDefinitions.Yellow());
         colors.add(ColorDefinitions.BlueViolet());
         colors.add(ColorDefinitions.Azure());

         heightWaypoints.clear();
         for (int i = 0; i < colors.size(); i++)
         {
            group.addChild(heightWaypoints.add().createSCS2YoGraphics("HeightWaypoint" + i, colors.get(i)));
         }
         heightWaypoints.clear();

         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredCoMPosition", desiredCoMPosition, 0.035, ColorDefinitions.Gold()));
      }
      return group;
   }
}
