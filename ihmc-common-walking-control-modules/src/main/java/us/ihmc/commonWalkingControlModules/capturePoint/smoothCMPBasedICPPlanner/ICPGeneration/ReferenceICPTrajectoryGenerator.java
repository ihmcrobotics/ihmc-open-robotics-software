package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Tim Seyde
 */

public class ReferenceICPTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final static boolean CONTINUOUSLY_ADJUST_FOR_ICP_DISCONTINUITY = true;

   private final static int FIRST_SEGMENT = 0;

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

   private final FramePoint3D icpPositionDesiredFinalCurrentSegment = new FramePoint3D();
   private final FramePoint3D icpPositionDesiredTerminal = new FramePoint3D();

   private final YoBoolean isInitialTransfer;

   private final YoBoolean continuouslyAdjustForICPContinuity;
   private final YoBoolean areICPDynamicsSatisfied;

   private final YoInteger totalNumberOfCMPSegments;
   private final YoInteger numberOfFootstepsToConsider;
   private final YoInteger currentSegmentIndex;

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

   private final SmoothCapturePointAdjustmentToolbox icpAdjustmentToolbox = new SmoothCapturePointAdjustmentToolbox();

   private final List<YoFramePoint3D> icpWaypoints = new ArrayList<>();
   private final List<YoDouble> icpSegmentDurations = new ArrayList<>();


   public ReferenceICPTrajectoryGenerator(String namePrefix, YoDouble omega0,
                                          YoInteger numberOfFootstepsToConsider,
                                          YoBoolean isInitialTransfer, boolean debug, YoVariableRegistry registry,
                                          YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.omega0 = omega0;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isInitialTransfer = isInitialTransfer;
      this.debug = debug;
      this.visualize = VISUALIZE && yoGraphicsListRegistry != null;

      areICPDynamicsSatisfied = new YoBoolean(namePrefix + "AreICPDynamicsSatisfied", registry);
      areICPDynamicsSatisfied.set(false);

      continuouslyAdjustForICPContinuity = new YoBoolean(namePrefix + "ContinuouslyAdjustForICPContinuity", registry);
      continuouslyAdjustForICPContinuity.set(CONTINUOUSLY_ADJUST_FOR_ICP_DISCONTINUITY);

      totalNumberOfCMPSegments = new YoInteger(namePrefix + "TotalNumberOfICPSegments", registry);

      startTimeOfCurrentPhase = new YoDouble(namePrefix + "StartTimeCurrentPhase", registry);
      localTimeInCurrentPhase = new YoDouble(namePrefix + "LocalTimeCurrentPhase", registry);
      localTimeInCurrentPhase.set(0.0);

      yoICPVelocityDynamicsCurrent = new YoFrameVector3D(namePrefix + "ICPVelocityDynamics", ReferenceFrame.getWorldFrame(), registry);

      currentSegmentIndex = new YoInteger(namePrefix + "CurrentSegment", registry);


      for (int i = 0; i < defaultSize; i++)
      {
         cmpDesiredFinalPositions.add(new FramePoint3D());
      }


      if (visualize)
      {
         ArtifactList icpWaypointList = new ArtifactList("ICP Waypoints");
         YoFramePoint3D waypointStart = new YoFramePoint3D("ICPWaypoint" + 0, ReferenceFrame.getWorldFrame(), registry);

         YoGraphicPosition waypointStartViz = new YoGraphicPosition("ICP Waypoint" + 0, waypointStart, POINT_SIZE, YoAppearance.Blue(), YoGraphicPosition.GraphicType.DIAMOND_WITH_CROSS);

         icpWaypointList.add(waypointStartViz.createArtifact());
         icpWaypoints.add(waypointStart);

         for (int i = 0; i < defaultSize; i++)
         {
            YoFramePoint3D waypoint = new YoFramePoint3D("ICPWaypoint" + (i + 1), ReferenceFrame.getWorldFrame(), registry);
            YoDouble segmentDurations = new YoDouble("ICPSegmentDuration" + i, registry);

            YoGraphicPosition waypointViz = new YoGraphicPosition("ICP Waypoint" + (i + 1), waypoint, POINT_SIZE, YoAppearance.Blue(), YoGraphicPosition.GraphicType.DIAMOND_WITH_CROSS);

            icpWaypointList.add(waypointViz.createArtifact());

            icpWaypoints.add(waypoint);
            icpSegmentDurations.add(segmentDurations);
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
      localTimeInCurrentPhase.set(0.0);

      icpPhaseEntryCornerPointIndices.resetQuick();
      icpPhaseExitCornerPointIndices.resetQuick();
   }

   public void resetCoPs()
   {
      copTrajectories.clear();
   }


   private void getICPInitialConditionsForAdjustment(double time, int currentSwingSegment)
   {
      getICPInitialConditionsForAdjustment(time, icpDesiredFinalPositionsFromCoPs, copTrajectories, currentSwingSegment, omega0.getDoubleValue());
   }

   private void getICPInitialConditionsForAdjustment(double localTime, List<FramePoint3D> exitCornerPointsFromCoPs, List<FrameTrajectory3D> copPolynomials3D,
                                                    int currentSwingSegment, double omega0)
   {
      calculatedInitialICPConditions.clear();
      if (currentSwingSegment < 0)
      { // called when in transfer
         FrameTrajectory3D firstCoPTrajectory = copPolynomials3D.get(0);
         for (int i = 0; i < firstCoPTrajectory.getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3DBasics icpQuantityInitialCondition = calculatedInitialICPConditions.add();

            firstCoPTrajectory.getDerivative(i, localTime, icpQuantityInitialCondition);
         }
      }
      else
      { // called when in swing
         FrameTrajectory3D copPolynomial3D = copPolynomials3D.get(currentSwingSegment);
         for (int i = 0; i < copPolynomials3D.get(0).getNumberOfCoefficients() / 2; i++)
         {
            FrameTuple3DBasics icpQuantityInitialCondition = calculatedInitialICPConditions.add();

            icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, localTime, i, copPolynomial3D,
                                                                            exitCornerPointsFromCoPs.get(currentSwingSegment), icpQuantityInitialCondition);
         }
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
         cmpTrajectories.addAll(transferCMPTrajectory.getSegments());
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

         SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = swingCMPTrajectory.getNumberOfSegments();
         cmpTrajectories.addAll(swingCMPTrajectory.getSegments());
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      int numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
      cmpTrajectories.addAll(transferCMPTrajectory.getSegments());
      totalNumberOfCMPSegments.add(numberOfCMPSegments);
      icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      if (visualize) {
         for (int i = 0; i < totalNumberOfCMPSegments.getIntegerValue(); i++) {
            icpSegmentDurations.get(i).set(cmpTrajectories.get(i).getDuration());
         }
      }

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
      cmpTrajectories.addAll(swingCMPTrajectory.getSegments());
      totalNumberOfCMPSegments.add(numberOfCMPSegments);
      icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
         cmpTrajectories.addAll(transferCMPTrajectory.getSegments());
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());

         swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = swingCMPTrajectory.getNumberOfSegments();
         cmpTrajectories.addAll(swingCMPTrajectory.getSegments());
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
         icpPhaseEntryCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
         icpPhaseExitCornerPointIndices.add(totalNumberOfCMPSegments.getIntegerValue());
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
      cmpTrajectories.addAll(transferCMPTrajectory.getSegments());
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
         copTrajectories.addAll(transferCoPTrajectory.getSegments());

         SegmentedFrameTrajectory3D swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
         copTrajectories.addAll(swingCoPTrajectory.getSegments());
      }

      SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);
      copTrajectories.addAll(transferCoPTrajectory.getSegments());

      icpToolbox.computeDesiredCornerPoints(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());

      if(isInitialTransfer.getBooleanValue())
      {
         getICPInitialConditionsForAdjustment(localTimeInCurrentPhase.getDoubleValue(), -1);
         setInitialConditionsForAdjustment();
      }
   }

   public void initializeForSwingFromCoPs(List<? extends SegmentedFrameTrajectory3D> transferCoPTrajectories,
                                          List<? extends SegmentedFrameTrajectory3D> swingCoPTrajectories)
   {
      resetCoPs();

      SegmentedFrameTrajectory3D swingCoPTrajectory = swingCoPTrajectories.get(0);
      copTrajectories.addAll(swingCoPTrajectory.getSegments());

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(stepIndex);
         copTrajectories.addAll(transferCoPTrajectory.getSegments());

         swingCoPTrajectory = swingCoPTrajectories.get(stepIndex);
         copTrajectories.addAll(swingCoPTrajectory.getSegments());
      }

      SegmentedFrameTrajectory3D transferCoPTrajectory = transferCoPTrajectories.get(numberOfSteps);
      copTrajectories.addAll(transferCoPTrajectory.getSegments());

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

      icpPositionDesiredTerminal.set(icpDesiredFinalPositions.get(cmpTrajectories.size() - 1));


      if (visualize)
      {
         for (int i = 0; i < icpWaypoints.size(); i++)
         {
            icpWaypoints.get(i).setToNaN();
         }

         int waypointIndex = 0;
         icpWaypoints.get(waypointIndex++).set(icpDesiredInitialPositions.get(0));

         for (int segmentIndex = 0; segmentIndex < icpDesiredFinalPositions.size(); segmentIndex++, waypointIndex++)
         {
            icpWaypoints.get(waypointIndex).set(icpDesiredFinalPositions.get(segmentIndex));
         }
      }
   }


   public void setInitialConditionsForAdjustment()
   {
      setInitialICPConditions.clear();
      for(int i = 0; i < calculatedInitialICPConditions.size(); i++)
         setInitialICPConditions.add().setIncludingFrame(calculatedInitialICPConditions.get(i));
   }

   public void adjustDesiredTrajectoriesForInitialSmoothing()
   {
      if ((isInitialTransfer.getBooleanValue() || (continuouslyAdjustForICPContinuity.getBooleanValue())) && copTrajectories.size() > 1)
      {
         icpAdjustmentToolbox.adjustDesiredTrajectoriesForInitialSmoothing3D(omega0.getDoubleValue(), copTrajectories, setInitialICPConditions,
                                                                             icpDesiredFinalPositionsFromCoPs.get(1));
         icpToolbox.computeDesiredCornerPoints3D(icpDesiredInitialPositionsFromCoPs, icpDesiredFinalPositionsFromCoPs, copTrajectories, omega0.getDoubleValue());
      }
   }

   @Override
   public void compute(double time)
   {
      if (cmpTrajectories.size() > 0)
      {
         localTimeInCurrentPhase.set(time - startTimeOfCurrentPhase.getDoubleValue());

         currentSegmentIndex.set(getCurrentSegmentIndex(localTimeInCurrentPhase.getDoubleValue(), cmpTrajectories));
         FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(currentSegmentIndex.getIntegerValue());
         getICPPositionDesiredFinalFromSegment(icpPositionDesiredFinalCurrentSegment, currentSegmentIndex.getIntegerValue());

         // ICP
         icpToolbox.computeDesiredCapturePointPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalCurrentSegment,
                                                       cmpPolynomial3D, icpPositionDesiredCurrent);
         icpToolbox.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalCurrentSegment,
                                                       cmpPolynomial3D, icpVelocityDesiredCurrent);
         icpToolbox.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(),
                                                           icpPositionDesiredFinalCurrentSegment, cmpPolynomial3D, icpAccelerationDesiredCurrent);

         int currentCoPSegmentIndex = getCurrentSegmentIndex(localTimeInCurrentPhase.getDoubleValue(), copTrajectories);
         getICPInitialConditionsForAdjustment(localTimeInCurrentPhase.getDoubleValue(), currentCoPSegmentIndex); // TODO: add controller dt for proper continuation
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

   private int getCurrentSegmentIndex(double timeInCurrentPhase, List<FrameTrajectory3D> trajectories)
   {
      int currentSegmentIndex = FIRST_SEGMENT;
      boolean notLastSegment = currentSegmentIndex < trajectories.size() - 1;
      while (!trajectories.get(currentSegmentIndex).timeIntervalContains(timeInCurrentPhase) && notLastSegment)
      {
         notLastSegment = currentSegmentIndex < trajectories.size() - 1;

         if (notLastSegment)
         {
            double currentEndTime = trajectories.get(currentSegmentIndex).getFinalTime();
            double nextStartTime = trajectories.get(currentSegmentIndex + 1).getInitialTime();

            boolean nextSegmentSkipsTime = Math.abs(nextStartTime - currentEndTime) > 1.0e-5;
            if (nextSegmentSkipsTime)
               return currentSegmentIndex;
         }

         currentSegmentIndex++;
      }

      return currentSegmentIndex;
   }


   public void getICPPositionDesiredFinalFromSegment(FramePoint3D icpPositionDesiredFinal, int segment)
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
         FramePoint3D icpPhaseEntryCornerPoint = icpDesiredInitialPositions.get(icpPhaseEntryCornerPointIndices.get(i));
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
         FramePoint3D icpPhaseExitCornerPoint = icpDesiredFinalPositions.get(icpPhaseExitCornerPointIndices.get(i) - 1);
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
