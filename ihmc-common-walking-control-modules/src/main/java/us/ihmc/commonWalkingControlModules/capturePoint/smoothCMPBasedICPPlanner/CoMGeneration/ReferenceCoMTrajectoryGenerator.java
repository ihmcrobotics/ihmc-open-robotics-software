package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
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

public class ReferenceCoMTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final static int FIRST_SEGMENT = 0;

   private final static int defaultSize = 100;

   private final List<FramePoint3D> comDesiredInitialPositions = new ArrayList<>(defaultSize);
   private final List<FrameVector3D> comDesiredInitialVelocities = new ArrayList<>(defaultSize);
   private final List<FrameVector3D> comDesiredInitialAccelerations = new ArrayList<>(defaultSize);
   private final List<FramePoint3D> comDesiredFinalPositions = new ArrayList<>(defaultSize);
   private final List<FrameVector3D> comDesiredFinalVelocities = new ArrayList<>(defaultSize);
   private final List<FrameVector3D> comDesiredFinalAccelerations = new ArrayList<>(defaultSize);

   private final List<FrameTrajectory3D> cmpTrajectories = new ArrayList<>(defaultSize);

   private List<? extends FramePoint3DReadOnly> icpDesiredFinalPositions = new ArrayList<>(defaultSize);

   private final FramePoint3D comPositionDesiredCurrent = new FramePoint3D();
   private final FrameVector3D comVelocityDesiredCurrent = new FrameVector3D();
   private final FrameVector3D comAccelerationDesiredCurrent = new FrameVector3D();

   private final FramePoint3D icpPositionDesiredFinalCurrentSegment = new FramePoint3D();
   private final FramePoint3D comPositionDesiredInitialCurrentSegment = new FramePoint3D();
   private final FrameVector3D comVelocityDesiredInitialCurrentSegment = new FrameVector3D();
   private final FrameVector3D comAccelerationDesiredInitialCurrentSegment = new FrameVector3D();

   private final FramePoint3D comDesiredPositionAtStartOfPhase = new FramePoint3D();
   private final FramePoint3D comDesiredVelocityAtStartOfPhase = new FramePoint3D();
   private final FramePoint3D comDesiredAccelerationAtStartOfPhase = new FramePoint3D();

   private final FramePoint3D comPositionDesiredFinalCurrentSegment = new FramePoint3D();

   private final YoInteger currentSegmentIndex;

   private final YoBoolean isInitialTransfer;
   private final YoBoolean isDoubleSupport;

   private final YoInteger totalNumberOfCMPSegments;
   private final YoInteger numberOfSegmentsInCurrentPhase;
   private final YoInteger numberOfFootstepsToConsider;

   private int numberOfFootstepsRegistered;

   private int numberOfSegmentsSwing0;
   private int numberOfSegmentsTransfer0;

   private final YoDouble omega0;
   private final YoDouble startTimeOfCurrentPhase;
   private final YoDouble localTimeInCurrentPhase;

   private final SmoothCoMIntegrationToolbox comToolbox = new SmoothCoMIntegrationToolbox();

   public ReferenceCoMTrajectoryGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider,
                                          YoBoolean isInitialTransfer, YoBoolean isDoubleSupport, YoRegistry registry)
   {
      this.omega0 = omega0;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isInitialTransfer = isInitialTransfer;
      this.isDoubleSupport = isDoubleSupport;

      totalNumberOfCMPSegments = new YoInteger("CoMGeneratorTotalNumberOfCMPSegments", registry);
      numberOfSegmentsInCurrentPhase = new YoInteger("CoMGeneratorNumberOfCMPSegmentsInCurrentPhase", registry);

      startTimeOfCurrentPhase = new YoDouble(namePrefix + "CoMGeneratorStartTimeCurrentPhase", registry);
      localTimeInCurrentPhase = new YoDouble(namePrefix + "CoMGeneratorLocalTimeCurrentPhase", registry);
      localTimeInCurrentPhase.set(0.0);

      currentSegmentIndex = new YoInteger(namePrefix + "CoMGeneratorCurrentSegment", registry);

      for (int i = 0; i < defaultSize; i++)
      {
         comDesiredInitialPositions.add(new FramePoint3D());
         comDesiredFinalPositions.add(new FramePoint3D());
         comDesiredInitialVelocities.add(new FrameVector3D());
         comDesiredFinalVelocities.add(new FrameVector3D());
         comDesiredInitialAccelerations.add(new FrameVector3D());
         comDesiredFinalAccelerations.add(new FrameVector3D());
      }
   }

   public void setNumberOfRegisteredSteps(int numberOfFootstepsRegistered)
   {
      this.numberOfFootstepsRegistered = numberOfFootstepsRegistered;
   }

   public void computeTrajectoryStartingFromTransfer(List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                                     List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories, List<? extends FramePoint3DReadOnly> icpDesiredFinalPositions)
   {
      reset();
      startTimeOfCurrentPhase.set(0.0);

      this.icpDesiredFinalPositions = icpDesiredFinalPositions;

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         int numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int i = 0; i < transferCMPTrajectory.getSegments().size(); i++)
            cmpTrajectories.add(transferCMPTrajectory.getSegments().get(i));
         totalNumberOfCMPSegments.add(numberOfCMPSegments);

         if (stepIndex == 0)
            numberOfSegmentsInCurrentPhase.set(numberOfCMPSegments);

         SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int i = 0; i < swingCMPTrajectory.getSegments().size(); i++)
            cmpTrajectories.add(swingCMPTrajectory.getSegments().get(i));
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      int numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int i = 0; i < transferCMPTrajectory.getSegments().size(); i++)
         cmpTrajectories.add(transferCMPTrajectory.getSegments().get(i));
      totalNumberOfCMPSegments.add(numberOfCMPSegments);

      if (numberOfSteps == 0)
         numberOfSegmentsInCurrentPhase.set(numberOfCMPSegments);

      numberOfSegmentsTransfer0 = transferCMPTrajectories.get(0).getNumberOfSegments();

      setCoMInitialConditions();
      initialize();
   }

   public void reset()
   {
      cmpTrajectories.clear();
      totalNumberOfCMPSegments.set(0);
      numberOfSegmentsInCurrentPhase.set(0);
      localTimeInCurrentPhase.set(0.0);
   }

   public void computeTrajectoryStartingFromSingleSupport(List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                                          List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories, List<? extends FramePoint3DReadOnly> icpDesiredFinalPositions)
   {
      reset();
      startTimeOfCurrentPhase.set(0.0);

      this.icpDesiredFinalPositions = icpDesiredFinalPositions;

      SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(0);
      int numberOfCMPSegments = swingCMPTrajectory.getNumberOfSegments();
      for (int i = 0; i < swingCMPTrajectory.getSegments().size(); i++)
         cmpTrajectories.add(swingCMPTrajectory.getSegments().get(i));
      totalNumberOfCMPSegments.add(numberOfCMPSegments);

      numberOfSegmentsInCurrentPhase.set(numberOfCMPSegments);

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int i = 0; i < transferCMPTrajectory.getSegments().size(); i++)
            cmpTrajectories.add(transferCMPTrajectory.getSegments().get(i));
         totalNumberOfCMPSegments.add(numberOfCMPSegments);

         swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         numberOfCMPSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int i = 0; i < swingCMPTrajectory.getSegments().size(); i++)
            cmpTrajectories.add(swingCMPTrajectory.getSegments().get(i));
         totalNumberOfCMPSegments.add(numberOfCMPSegments);
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      numberOfCMPSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int i = 0; i < transferCMPTrajectory.getSegments().size(); i++)
         cmpTrajectories.add(transferCMPTrajectory.getSegments().get(i));
      totalNumberOfCMPSegments.add(numberOfCMPSegments);

      numberOfSegmentsSwing0 = swingCMPTrajectories.get(0).getNumberOfSegments();

      setCoMInitialConditions();
      initialize();
   }

   public void initializeForSwingOrTransfer()
   {
      comDesiredPositionAtStartOfPhase.set(comPositionDesiredCurrent);
      comDesiredVelocityAtStartOfPhase.set(comVelocityDesiredCurrent);
      comDesiredAccelerationAtStartOfPhase.set(comAccelerationDesiredCurrent);
   }

   @Override
   public void initialize()
   {
      comToolbox.computeDesiredCenterOfMassCornerData(icpDesiredFinalPositions, comDesiredInitialPositions,
                                                      comDesiredFinalPositions, comDesiredInitialVelocities, comDesiredFinalVelocities,
                                                      comDesiredInitialAccelerations, comDesiredFinalAccelerations, cmpTrajectories,
                                                      comPositionDesiredInitialCurrentSegment, comVelocityDesiredInitialCurrentSegment,
                                                      comAccelerationDesiredInitialCurrentSegment, omega0.getDoubleValue());
   }

   private void setCoMInitialConditions()
   {
      if (isInitialTransfer.getBooleanValue())
      {
         FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(0);
         cmpPolynomial3D.compute(cmpPolynomial3D.getInitialTime());
         comPositionDesiredInitialCurrentSegment.set(cmpPolynomial3D.getFramePosition());
         comVelocityDesiredInitialCurrentSegment.setToZero();
         comAccelerationDesiredInitialCurrentSegment.setToZero();
      }
      else
      {
         comPositionDesiredInitialCurrentSegment.set(comDesiredPositionAtStartOfPhase);
         comVelocityDesiredInitialCurrentSegment.set(comDesiredVelocityAtStartOfPhase);
         comAccelerationDesiredInitialCurrentSegment.set(comDesiredAccelerationAtStartOfPhase);
      }
   }

   @Override
   public void compute(double time)
   {
      localTimeInCurrentPhase.set(time - startTimeOfCurrentPhase.getDoubleValue());

      currentSegmentIndex.set(getCurrentSegmentIndex(localTimeInCurrentPhase.getDoubleValue(), cmpTrajectories));
      FrameTrajectory3D cmpPolynomial3D = cmpTrajectories.get(currentSegmentIndex.getIntegerValue());
      getICPPositionDesiredFinalFromSegment(icpPositionDesiredFinalCurrentSegment, currentSegmentIndex.getIntegerValue());
      getPositionDesiredInitialFromSegment(comPositionDesiredInitialCurrentSegment, currentSegmentIndex.getIntegerValue());
      getPositionDesiredFinalFromSegment(comPositionDesiredFinalCurrentSegment, currentSegmentIndex.getIntegerValue());

      // CoM
      comToolbox.computeDesiredCenterOfMassPositionVelocityAcceleration(omega0.getValue(), localTimeInCurrentPhase.getValue(),
                                                                        icpPositionDesiredFinalCurrentSegment, comPositionDesiredInitialCurrentSegment,
                                                                        cmpPolynomial3D, comPositionDesiredCurrent, comVelocityDesiredCurrent,
                                                                        comAccelerationDesiredCurrent);
   }

   private int getCurrentSegmentIndex(double timeInCurrentPhase, List<FrameTrajectory3D> trajectories)
   {
      int currentSegmentIndex = numberOfSegmentsInCurrentPhase.getIntegerValue() - 1;
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

   public void getPositionDesiredInitialFromSegment(FixedFramePoint3DBasics comPositionDesiredInitial, int segment)
   {
      comPositionDesiredInitial.set(comDesiredInitialPositions.get(segment));
   }

   public void getPositionDesiredFinalFromSegment(FixedFramePoint3DBasics comPositionDesiredFinal, int segment)
   {
      comPositionDesiredFinal.set(comDesiredFinalPositions.get(segment));
   }

   public void getFinalCoMPositionInSwing(FixedFramePoint3DBasics comPositionDesiredFinalToPack)
   {
      if (isDoubleSupport.getBooleanValue())
      {
         getPositionDesiredFinalFromSegment(comPositionDesiredFinalToPack, numberOfSegmentsTransfer0 + numberOfSegmentsSwing0 - 1);
      }
      else
      {
         getPositionDesiredFinalFromSegment(comPositionDesiredFinalToPack, numberOfSegmentsSwing0 - 1);
      }
   }

   public void getFinalCoMPositionInTransfer(FixedFramePoint3DBasics comPositionDesiredFinalToPack)
   {
      if (isDoubleSupport.getBooleanValue())
      {
         getPositionDesiredFinalFromSegment(comPositionDesiredFinalToPack, numberOfSegmentsTransfer0 - 1);
      }
      else
      {
         getPositionDesiredFinalFromSegment(comPositionDesiredFinalToPack, numberOfSegmentsSwing0 + numberOfSegmentsTransfer0 - 1);
      }
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.set(comPositionDesiredCurrent);
   }

   public void getPosition(YoFramePoint3D positionToPack)
   {
      positionToPack.set(comPositionDesiredCurrent);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.set(comVelocityDesiredCurrent);
   }

   public void getVelocity(YoFrameVector3D velocityToPack)
   {
      velocityToPack.set(comVelocityDesiredCurrent);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.set(comAccelerationDesiredCurrent);
   }

   public void getAcceleration(YoFrameVector3D accelerationToPack)
   {
      accelerationToPack.set(comAccelerationDesiredCurrent);
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

   public List<? extends FramePoint3DReadOnly> getCoMPositionDesiredInitialList()
   {
      return comDesiredInitialPositions;
   }

   public List<? extends FramePoint3DReadOnly> getCoMPositionDesiredFinalList()
   {
      return comDesiredFinalPositions;
   }

   public List<? extends FrameVector3DReadOnly> getCoMVelocityDesiredInitialList()
   {
      return comDesiredInitialVelocities;
   }

   public List<? extends FrameVector3DReadOnly> getCoMVelocityDesiredFinalList()
   {
      return comDesiredFinalVelocities;
   }

   public List<? extends FrameVector3DReadOnly> getCoMAccelerationDesiredInitialList()
   {
      return comDesiredInitialAccelerations;
   }

   public List<? extends FrameVector3DReadOnly> getCoMAccelerationDesiredFinalList()
   {
      return comDesiredFinalAccelerations;
   }
}
