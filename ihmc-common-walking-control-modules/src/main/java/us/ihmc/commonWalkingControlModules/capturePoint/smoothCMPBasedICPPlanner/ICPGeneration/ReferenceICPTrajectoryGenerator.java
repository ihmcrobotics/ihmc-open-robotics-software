package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DBasics;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * @author Tim Seyde
 */

public class ReferenceICPTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final static boolean CONTINUOUSLY_ADJUST_FOR_ICP_DISCONTINUITY = true;

   private final static int defaultSize = 300;

   private final static double POINT_SIZE = 0.005;
   private static final boolean VISUALIZE = false;

   private final boolean debug;
   private final boolean visualize;

   private final List<FramePoint3D> cmpDesiredFinalPositions = new ArrayList<>();

   private final RecyclingArrayList<FramePoint3D> icpDesiredInitialPositions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> icpDesiredFinalPositions = new RecyclingArrayList<>(FramePoint3D::new);

   private final RecyclingArrayList<FramePoint3D> icpDesiredInitialPositionsFromCoPs = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FramePoint3D> icpDesiredFinalPositionsFromCoPs = new RecyclingArrayList<>(FramePoint3D::new);

   private final FramePoint3D icpPositionDesiredCurrent = new FramePoint3D();
   private final FrameVector3D icpVelocityDesiredCurrent = new FrameVector3D();
   private final FrameVector3D icpAccelerationDesiredCurrent = new FrameVector3D();
   private final FrameVector3D icpVelocityDynamicsCurrent = new FrameVector3D();
   private final YoFrameVector3D yoICPVelocityDynamicsCurrent;

   private final YoBoolean copWaypointsWereAdjusted;
   private final YoBoolean isInitialTransfer;
   private final YoBoolean isStanding;

   private final YoBoolean continuouslyAdjustForICPContinuity;
   private final YoBoolean areICPDynamicsSatisfied;

   private final YoInteger totalNumberOfCoPSegments;
   private final YoInteger totalNumberOfCMPSegments;
   private final YoInteger numberOfFootstepsToConsider;

   private final YoInteger currentCMPSegmentIndex;
   private final YoInteger currentCoPSegmentIndex;

   private final YoInteger numberOfCoPSegmentsInCurrentPhase;
   private final YoInteger numberOfCMPSegmentsInCurrentPhase;

   private final YoDouble startTimeOfCurrentPhase;
   private final YoDouble localTimeInCurrentPhase;
   private final YoDouble omega0;

   private int numberOfFootstepsRegistered;

   private final List<FrameTrajectory3D> copTrajectories = new ArrayList<>();
   private final List<FrameTrajectory3D> cmpTrajectories = new ArrayList<>();

   private final TIntArrayList icpPhaseExitCornerPointIndices = new TIntArrayList();
   private final TIntArrayList icpPhaseEntryCornerPointIndices = new TIntArrayList();

   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();

   private final RecyclingArrayList<FrameTuple3DBasics> calculatedInitialICPConditions = new RecyclingArrayList<>(FramePoint3D::new);
   private final RecyclingArrayList<FrameTuple3DBasics> setInitialICPConditions = new RecyclingArrayList<>(FramePoint3D::new);

   private final YoFramePoint3D initialICPConditionForContinuity;
   private final YoFramePoint3D finalICPConditionForContinuity;

   private final YoFramePoint3D endOfPhaseICPPosition;
   private final YoFramePoint3D endOfSegmentICPPosition;

   private final SmoothCapturePointAdjustmentToolbox icpAdjustmentToolbox = new SmoothCapturePointAdjustmentToolbox();

   private final List<YoFramePoint3D> icpWaypoints = new ArrayList<>();
   private final List<YoDouble> icpSegmentDurations = new ArrayList<>();
   private final List<YoDouble> icpSegmentStartTimes = new ArrayList<>();
   private final List<YoDouble> icpSegmentEndTimes = new ArrayList<>();

   public ReferenceICPTrajectoryGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider, YoBoolean isInitialTransfer,
                                          YoBoolean isStanding, boolean debug, YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.omega0 = omega0;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isInitialTransfer = isInitialTransfer;
      this.isStanding = isStanding;
      this.debug = debug;
      this.visualize = VISUALIZE && yoGraphicsListRegistry != null;

      areICPDynamicsSatisfied = new YoBoolean(namePrefix + "AreICPDynamicsSatisfied", registry);
      areICPDynamicsSatisfied.set(false);

      copWaypointsWereAdjusted = new YoBoolean("CoPWaypointsWereAdjusted", registry);

      continuouslyAdjustForICPContinuity = new YoBoolean(namePrefix + "ContinuouslyAdjustForICPContinuity", registry);
      continuouslyAdjustForICPContinuity.set(CONTINUOUSLY_ADJUST_FOR_ICP_DISCONTINUITY);

      totalNumberOfCoPSegments = new YoInteger(namePrefix + "TotalNumberOfCoPSegments", registry);
      totalNumberOfCMPSegments = new YoInteger(namePrefix + "TotalNumberOfCMPSegments", registry);

      startTimeOfCurrentPhase = new YoDouble(namePrefix + "StartTimeCurrentPhase", registry);
      localTimeInCurrentPhase = new YoDouble(namePrefix + "LocalTimeCurrentPhase", registry);
      localTimeInCurrentPhase.set(0.0);

      yoICPVelocityDynamicsCurrent = new YoFrameVector3D(namePrefix + "ICPVelocityDynamics", ReferenceFrame.getWorldFrame(), registry);
      initialICPConditionForContinuity = new YoFramePoint3D("InitialICPConditionForContinuity", ReferenceFrame.getWorldFrame(), registry);
      finalICPConditionForContinuity = new YoFramePoint3D("FinalICPConditionForContinuity", ReferenceFrame.getWorldFrame(), registry);

      endOfPhaseICPPosition = new YoFramePoint3D("EndOfPhaseICPPosition", ReferenceFrame.getWorldFrame(), registry);
      endOfSegmentICPPosition = new YoFramePoint3D("EndOfSegmentICPPosition", ReferenceFrame.getWorldFrame(), registry);

      currentCMPSegmentIndex = new YoInteger(namePrefix + "CurrentCMPSegment", registry);
      currentCoPSegmentIndex = new YoInteger(namePrefix + "CurrentCoPSegment", registry);
      numberOfCMPSegmentsInCurrentPhase = new YoInteger("NumberOfCMPSegmentsInCurrentPhase", registry);
      numberOfCoPSegmentsInCurrentPhase = new YoInteger("NumberOfCoPSegmentsInCurrentPhase", registry);

      for (int i = 0; i < defaultSize; i++)
      {
         cmpDesiredFinalPositions.add(new FramePoint3D());
      }

      if (visualize)
      {
         ArtifactList icpWaypointList = new ArtifactList("ICP Waypoints");
         YoFramePoint3D waypointStart = new YoFramePoint3D("ICPWaypoint" + 0, ReferenceFrame.getWorldFrame(), registry);

         YoGraphicPosition waypointStartViz = new YoGraphicPosition("ICP Waypoint" + 0, waypointStart, POINT_SIZE, YoAppearance.Blue(),
                                                                    YoGraphicPosition.GraphicType.DIAMOND_WITH_CROSS);

         icpWaypointList.add(waypointStartViz.createArtifact());
         icpWaypoints.add(waypointStart);

         for (int i = 0; i < defaultSize; i++)
         {
            YoFramePoint3D waypoint = new YoFramePoint3D("ICPWaypoint" + (i + 1), ReferenceFrame.getWorldFrame(), registry);
            YoDouble segmentDurations = new YoDouble("ICPSegmentDuration" + i, registry);
            YoDouble segmentStartTimes = new YoDouble("ICPSegmentStartTime" + i, registry);
            YoDouble segmentEndTimes = new YoDouble("ICPSegmentEndTime" + i, registry);

            YoGraphicPosition waypointViz = new YoGraphicPosition("ICP Waypoint" + (i + 1), waypoint, POINT_SIZE, YoAppearance.Blue(),
                                                                  YoGraphicPosition.GraphicType.DIAMOND_WITH_CROSS);

            icpWaypointList.add(waypointViz.createArtifact());

            icpWaypoints.add(waypoint);
            icpSegmentDurations.add(segmentDurations);
            icpSegmentStartTimes.add(segmentStartTimes);
            icpSegmentEndTimes.add(segmentEndTimes);
         }

         yoGraphicsListRegistry.registerArtifactList(icpWaypointList);
      }
   }

   public void setNumberOfRegisteredSteps(int numberOfFootstepsRegistered)
   {
      this.numberOfFootstepsRegistered = numberOfFootstepsRegistered;
   }

   public void reset()
   {
      cmpTrajectories.clear();
      totalNumberOfCMPSegments.set(0);
      numberOfCMPSegmentsInCurrentPhase.set(0);
      localTimeInCurrentPhase.set(0.0);

      icpPhaseEntryCornerPointIndices.resetQuick();
      icpPhaseExitCornerPointIndices.resetQuick();

      if (visualize)
      {
         for (int i = 0; i < icpWaypoints.size(); i++)
         {
            icpWaypoints.get(i).setToNaN();
         }

         for (int i = 0; i < icpSegmentDurations.size(); i++)
         {
            icpSegmentDurations.get(i).setToNaN();
            icpSegmentStartTimes.get(i).setToNaN();
            icpSegmentEndTimes.get(i).setToNaN();
         }

      }
   }

   public void resetCoPs()
   {
      copTrajectories.clear();
      totalNumberOfCoPSegments.set(0);
      numberOfCoPSegmentsInCurrentPhase.set(0);
      copWaypointsWereAdjusted.set(false);
   }

   private void getICPInitialConditionsForAdjustmentFromCMPs(double time, int currentSwingSegment)
   {
      getICPInitialConditionsForAdjustment(time, icpDesiredFinalPositions, cmpTrajectories, currentSwingSegment);
   }

   private void getICPInitialConditionsForAdjustmentFromCoPs(double time, int currentSwingSegment)
   {
      getICPInitialConditionsForAdjustment(time, icpDesiredFinalPositionsFromCoPs, copTrajectories, currentSwingSegment);
   }

   private void getICPInitialConditionsForAdjustment(double localTime, List<FramePoint3D> exitCornerPoints, List<FrameTrajectory3D> trajectories,
                                                     int currentTrajectoryIndex)
   {
      calculatedInitialICPConditions.clear();
      FrameTrajectory3D firstTrajectory = trajectories.get(0);
      for (int order = 0; order < firstTrajectory.getNumberOfCoefficients() / 2; order++)
      {
         FrameTuple3DBasics icpQuantityInitialCondition = calculatedInitialICPConditions.add();
         icpToolbox
               .calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0.getDoubleValue(), localTime, order, trajectories.get(currentTrajectoryIndex),
                                                                     exitCornerPoints.get(currentTrajectoryIndex), icpQuantityInitialCondition);
      }
   }

   public void initializeForTransfer(double initialTime, List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                     List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);

      icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         int numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int i = 0; i < transferCMPTrajectory.getSegments().size(); i++)
            cmpTrajectories.add(transferCMPTrajectory.getSegments().get(i));
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

         if (stepIndex == 0)
            numberOfCMPSegmentsInCurrentPhase.set(numberOfCMPSegments);

         SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int i = 0; i < swingCMPTrajectory.getSegments().size(); i++)
            cmpTrajectories.add(swingCMPTrajectory.getSegments().get(i));
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      int numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int i = 0; i < transferCMPTrajectory.getSegments().size(); i++)
         cmpTrajectories.add(transferCMPTrajectory.getSegments().get(i));
      totalNumberOfCMPSegments.add(numberOfCMPSegments);
      icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      if (numberOfSteps == 0)
         numberOfCMPSegmentsInCurrentPhase.set(numberOfCMPSegments);

      initialize();
   }

   public void initializeForSwing(double initialTime, List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                  List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);

      icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(0);
      int numberOfCMPSegments = swingCMPTrajectory.getNumberOfSegments();
      for (int i = 0; i < swingCMPTrajectory.getSegments().size(); i++)
         cmpTrajectories.add(swingCMPTrajectory.getSegments().get(i));
      totalNumberOfCMPSegments.add(numberOfCMPSegments);
      icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      numberOfCMPSegmentsInCurrentPhase.set(numberOfCMPSegments);

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int i = 0; i < transferCMPTrajectory.getSegments().size(); i++)
            cmpTrajectories.add(transferCMPTrajectory.getSegments().get(i));
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

         swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int i = 0; i < swingCMPTrajectory.getSegments().size(); i++)
            cmpTrajectories.add(swingCMPTrajectory.getSegments().get(i));
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int i = 0; i < transferCMPTrajectory.getSegments().size(); i++)
         cmpTrajectories.add(transferCMPTrajectory.getSegments().get(i));
      totalNumberOfCMPSegments.add(numberOfCMPSegments);
      icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      initialize();
   }

   public void initializeForTransferFromCoPs(List<? extends SegmentedFrameTrajectory3D> transferCoPTrajectories,
                                             List<? extends SegmentedFrameTrajectory3D> swingCoPTrajectories)
   {
      resetCoPs();

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
         int numberOfCoPSegments = transferCoPTrajectory.getNumberOfSegments();
         for (int i = 0; i < transferCoPTrajectory.getSegments().size(); i++)
            copTrajectories.add(transferCoPTrajectory.getSegments().get(i));
         totalNumberOfCoPSegments.add(numberOfCoPSegments);

         if (stepIndex == 0)
            numberOfCoPSegmentsInCurrentPhase.set(numberOfCoPSegments);

         SegmentedFrameTrajectory3D swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
         numberOfCoPSegments = swingCoPTrajectory.getNumberOfSegments();
         for (int i = 0; i < swingCoPTrajectory.getSegments().size(); i++)
            copTrajectories.add(swingCoPTrajectory.getSegments().get(i));
         totalNumberOfCoPSegments.add(numberOfCoPSegments);
      }

      SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);
      int numberOfCoPSegments = transferCoPTrajectory.getNumberOfSegments();
      for (int i = 0; i < transferCoPTrajectory.getSegments().size(); i++)
         copTrajectories.add(transferCoPTrajectory.getSegments().get(i));
      totalNumberOfCoPSegments.add(numberOfCoPSegments);
      if (numberOfSteps == 0)
         numberOfCoPSegmentsInCurrentPhase.set(numberOfCoPSegments);

      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());

      if (isInitialTransfer.getBooleanValue() && isStanding.getBooleanValue())
      {
         getICPInitialConditionsForAdjustmentFromCoPs(localTimeInCurrentPhase.getDoubleValue(), 0);
         setInitialConditionsForAdjustment();
      }
   }

   public void initializeForSwingFromCoPs(List<? extends SegmentedFrameTrajectory3D> transferCoPTrajectories,
                                          List<? extends SegmentedFrameTrajectory3D> swingCoPTrajectories)
   {
      resetCoPs();

      SegmentedFrameTrajectory3D swingCoPTrajectory = swingCoPTrajectories.get(0);
      for (int i = 0; i < swingCoPTrajectory.getSegments().size(); i++)
         copTrajectories.add(swingCoPTrajectory.getSegments().get(i));
      int numberOfCoPSegments = swingCoPTrajectory.getNumberOfSegments();
      totalNumberOfCoPSegments.add(numberOfCoPSegments);
      numberOfCoPSegmentsInCurrentPhase.set(numberOfCoPSegments);

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
         for (int i = 0; i < transferCoPTrajectory.getSegments().size(); i++)
            copTrajectories.add(transferCoPTrajectory.getSegments().get(i));
         totalNumberOfCoPSegments.add(transferCoPTrajectory.getNumberOfSegments());

         swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
         for (int i = 0; i < swingCoPTrajectory.getSegments().size(); i++)
            copTrajectories.add(swingCoPTrajectory.getSegments().get(i));
         totalNumberOfCoPSegments.add(swingCoPTrajectory.getNumberOfSegments());
      }

      SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);
      for (int i = 0; i < transferCoPTrajectory.getSegments().size(); i++)
         copTrajectories.add(transferCoPTrajectory.getSegments().get(i));
      totalNumberOfCoPSegments.add(transferCoPTrajectory.getNumberOfSegments());

      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());
   }

   @Override
   public void initialize()
   {
      if (isInitialTransfer.getBooleanValue())
      {
         FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(0);
         cmpPolynomial3D.compute(cmpPolynomial3D.getInitialTime());
      }

      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositions, icpDesiredFinalPositions, cmpTrajectories, omega0.getDoubleValue());

      endOfPhaseICPPosition.set(icpDesiredFinalPositions.get(cmpTrajectories.size() - 1));

      if (visualize)
      {
         int waypointIndex = 0;
         icpWaypoints.get(waypointIndex++).set(icpDesiredInitialPositions.get(0));

         for (int segmentIndex = 0; segmentIndex < icpDesiredFinalPositions.size(); segmentIndex++, waypointIndex++)
         {
            icpWaypoints.get(waypointIndex).set(icpDesiredFinalPositions.get(segmentIndex));
         }

         for (int i = 0; i < totalNumberOfCMPSegments.getIntegerValue(); i++)
         {
            icpSegmentDurations.get(i).set(cmpTrajectories.get(i).getDuration());
            icpSegmentStartTimes.get(i).set(cmpTrajectories.get(i).getInitialTime());
            icpSegmentEndTimes.get(i).set(cmpTrajectories.get(i).getFinalTime());
         }

         endOfSegmentICPPosition.setToNaN();
         endOfPhaseICPPosition.setToNaN();

         initialICPConditionForContinuity.setToNaN();
         finalICPConditionForContinuity.setToNaN();
      }
   }

   public void setInitialConditionsForAdjustment()
   {
      setInitialICPConditions.clear();

      for (int i = 0; i < calculatedInitialICPConditions.size(); i++)
         setInitialICPConditions.add().setIncludingFrame(calculatedInitialICPConditions.get(i));

      if (calculatedInitialICPConditions.size() > 0)
         initialICPConditionForContinuity.set(calculatedInitialICPConditions.get(0));
      else
         initialICPConditionForContinuity.setToNaN();
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing()
   {
      if ((isInitialTransfer.getBooleanValue() || (continuouslyAdjustForICPContinuity.getBooleanValue())) && copTrajectories.size() > 1)
      {
         finalICPConditionForContinuity.set(icpDesiredFinalPositionsFromCoPs.get(1));
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing3D(omega0.getDoubleValue(), copTrajectories, setInitialICPConditions,
                                                                             finalICPConditionForContinuity);
         icpToolbox
               .computeDesiredCornerPoints3D(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());
         copWaypointsWereAdjusted.set(true);
      }
   }

   @Override
   public void compute(double time)
   {
      if (cmpTrajectories.size() > 0)
      {
         localTimeInCurrentPhase.set(time - startTimeOfCurrentPhase.getDoubleValue());

         currentCMPSegmentIndex
               .set(getCurrentSegmentIndex(localTimeInCurrentPhase.getDoubleValue(), numberOfCMPSegmentsInCurrentPhase.getIntegerValue(), cmpTrajectories));
         FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(currentCMPSegmentIndex.getIntegerValue());
         getICPPositionDesiredFinalFromSegment(endOfSegmentICPPosition, currentCMPSegmentIndex.getIntegerValue());

         // ICP
         icpToolbox
               .computeDesiredCapturePointPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), endOfSegmentICPPosition, cmpPolynomial3D,
                                                   icpPositionDesiredCurrent);
         icpToolbox
               .computeDesiredCapturePointVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), endOfSegmentICPPosition, cmpPolynomial3D,
                                                   icpVelocityDesiredCurrent);
         icpToolbox.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), endOfSegmentICPPosition,
                                                           cmpPolynomial3D, icpAccelerationDesiredCurrent);

         currentCoPSegmentIndex
               .set(getCurrentSegmentIndex(localTimeInCurrentPhase.getDoubleValue(), numberOfCoPSegmentsInCurrentPhase.getIntegerValue(), copTrajectories));
         getICPInitialConditionsForAdjustmentFromCoPs(localTimeInCurrentPhase.getDoubleValue(),
                                                      currentCoPSegmentIndex.getIntegerValue()); // TODO: add controller dt for proper continuation
         if (debug)
            checkICPDynamics(localTimeInCurrentPhase.getDoubleValue(), icpVelocityDesiredCurrent, icpPositionDesiredCurrent, cmpPolynomial3D);

      }
   }

   private void checkICPDynamics(double time, FrameVector3D icpVelocityDesiredCurrent, FramePoint3D icpPositionDesiredCurrent,
                                 FrameTrajectory3D cmpPolynomial3D)
   {
      cmpPolynomial3D.compute(time);

      icpVelocityDynamicsCurrent.sub(icpPositionDesiredCurrent, cmpPolynomial3D.getFramePosition());
      icpVelocityDynamicsCurrent.scale(omega0.getDoubleValue());

      yoICPVelocityDynamicsCurrent.set(icpVelocityDynamicsCurrent);

      areICPDynamicsSatisfied.set(icpVelocityDesiredCurrent.epsilonEquals(icpVelocityDynamicsCurrent, 10e-5));
   }

   private int getCurrentSegmentIndex(double timeInCurrentPhase, int numberOfSegmentsInCurrentPhase, List<FrameTrajectory3D> trajectories)
   {
      int currentSegmentIndex = numberOfSegmentsInCurrentPhase - 1;
      if (currentSegmentIndex < 0)
      {
         return 0;
      }

      while (timeInCurrentPhase < trajectories.get(currentSegmentIndex).getInitialTime() - 1e-6 && currentSegmentIndex > 0)
      {
         currentSegmentIndex--;
      }

      return currentSegmentIndex;
   }

   public void getICPPositionDesiredFinalFromSegment(FixedFramePoint3DBasics icpPositionDesiredFinal, int segment)
   {
      icpPositionDesiredFinal.set(icpDesiredFinalPositions.get(segment));
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.set(icpPositionDesiredCurrent);
   }

   public void getPosition(YoFramePoint3D positionToPack)
   {
      positionToPack.set(icpPositionDesiredCurrent);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredCurrent);
   }

   public void getVelocity(YoFrameVector3D velocityToPack)
   {
      velocityToPack.set(icpVelocityDesiredCurrent);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.set(icpAccelerationDesiredCurrent);
   }

   public void getAcceleration(YoFrameVector3D accelerationToPack)
   {
      accelerationToPack.set(icpAccelerationDesiredCurrent);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void getLinearData(YoFramePoint3D positionToPack, YoFrameVector3D velocityToPack, YoFrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {

   }

   @Override
   public void hideVisualization()
   {

   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   public List<? extends FramePoint3DReadOnly> getICPPositionDesiredInitialList()
   {
      return icpDesiredInitialPositions;
   }

   public List<? extends FramePoint3DReadOnly> getICPPositionDesiredFinalList()
   {
      return icpDesiredFinalPositions;
   }

   public void getICPPhaseEntryCornerPoints(List<? extends FixedFramePoint3DBasics> icpPhaseEntryCornerPointsToPack)
   {
      int i = 0;
      for (; i < icpPhaseEntryCornerPointIndices.size(); i++)
      {
         FramePoint3D icpPhaseEntryCornerPoint = icpDesiredInitialPositions.getAndGrowIfNeeded(icpPhaseEntryCornerPointIndices.get(i));
         icpPhaseEntryCornerPointsToPack.get(i).set(icpPhaseEntryCornerPoint);
         icpPhaseEntryCornerPointsToPack.get(i).add(0.0, 0.0, 0.03);
      }
      for (; i < icpPhaseEntryCornerPointsToPack.size(); i++)
         icpPhaseEntryCornerPointsToPack.get(i).setToNaN();

   }

   public void getICPPhaseExitCornerPoints(List<? extends FixedFramePoint3DBasics> icpPhaseExitCornerPointsToPack)
   {
      int i = 0;
      for (; i < icpPhaseExitCornerPointIndices.size(); i++)
      {
         FramePoint3D icpPhaseExitCornerPoint = icpDesiredFinalPositions.getAndGrowIfNeeded(icpPhaseExitCornerPointIndices.get(i) - 1);
         icpPhaseExitCornerPointsToPack.get(i).set(icpPhaseExitCornerPoint);
         icpPhaseExitCornerPointsToPack.get(i).add(0.0, 0.0, 0.05);
      }
      for (; i < icpPhaseExitCornerPointsToPack.size(); i++)
         icpPhaseExitCornerPointsToPack.get(i).setToNaN();
   }

   public List<? extends FramePoint3DReadOnly> getICPPositonFromCoPDesiredFinalList()
   {
      return icpDesiredFinalPositionsFromCoPs;
   }

   public int getTotalNumberOfSegments()
   {
      return totalNumberOfCMPSegments.getIntegerValue();
   }
}
