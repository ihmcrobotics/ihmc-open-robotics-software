package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

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

   private List<FramePoint3D> icpDesiredFinalPositions = new ArrayList<>(defaultSize);

   private final FramePoint3D comPositionDesiredCurrent = new FramePoint3D();
   private final FrameVector3D comVelocityDesiredCurrent = new FrameVector3D();
   private final FrameVector3D comAccelerationDesiredCurrent = new FrameVector3D();

   private final FramePoint3D icpPositionDesiredFinalCurrentSegment = new FramePoint3D();
   private final FramePoint3D comPositionDesiredInitialCurrentSegment = new FramePoint3D();
   private final FrameVector3D comVelocityDesiredInitialCurrentSegment = new FrameVector3D();
   private final FrameVector3D comAccelerationDesiredInitialCurrentSegment = new FrameVector3D();

   private final FramePoint3D comPositionDesiredFinalCurrentSegment = new FramePoint3D();

   private final YoInteger currentSegmentIndex;

   private final YoBoolean isInitialTransfer;
   private final YoBoolean isDoubleSupport;

   private final YoInteger totalNumberOfCMPSegments;
   private final YoInteger numberOfFootstepsToConsider;

   private int numberOfFootstepsRegistered;

   private int numberOfSegmentsSwing0;
   private int numberOfSegmentsTransfer0;

   private final YoDouble omega0;
   private final YoDouble startTimeOfCurrentPhase;
   private final YoDouble localTimeInCurrentPhase;

   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();
   private final SmoothCoMIntegrationToolbox comToolbox = new SmoothCoMIntegrationToolbox(icpToolbox);

   public ReferenceCoMTrajectoryGenerator(String namePrefix, YoDouble omega0, YoInteger numberOfFootstepsToConsider,
                                          YoBoolean isInitialTransfer, YoBoolean isDoubleSupport, YoVariableRegistry registry)
   {
      this.omega0 = omega0;
      this.numberOfFootstepsToConsider = numberOfFootstepsToConsider;
      this.isInitialTransfer = isInitialTransfer;
      this.isDoubleSupport = isDoubleSupport;

      totalNumberOfCMPSegments = new YoInteger(namePrefix + "CoMGeneratorTotalNumberOfICPSegments", registry);

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

   public void initializeForTransfer(double initialTime, List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                     List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories, List<FramePoint3D> icpDesiredFinalPositions)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);

      this.icpDesiredFinalPositions = icpDesiredFinalPositions;

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 0; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }

         SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      int cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }

      numberOfSegmentsTransfer0 = transferCMPTrajectories.get(0).getNumberOfSegments();
      initialize();
   }

   public void reset()
   {
      cmpTrajectories.clear();
      totalNumberOfCMPSegments.set(0);
      localTimeInCurrentPhase.set(0.0);
   }

   public void initializeForSwing(double initialTime, List<? extends SegmentedFrameTrajectory3D> transferCMPTrajectories,
                                  List<? extends SegmentedFrameTrajectory3D> swingCMPTrajectories, List<FramePoint3D> icpDesiredFinalPositions)
   {
      reset();
      startTimeOfCurrentPhase.set(initialTime);

      this.icpDesiredFinalPositions = icpDesiredFinalPositions;

      SegmentedFrameTrajectory3D swingCMPTrajectory = swingCMPTrajectories.get(0);
      int cmpSegments = swingCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }

      int numberOfSteps = Math.min(numberOfFootstepsRegistered, numberOfFootstepsToConsider.getIntegerValue());
      for (int stepIndex = 1; stepIndex < numberOfSteps; stepIndex++)
      {
         SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(stepIndex);
         cmpSegments = transferCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }

         swingCMPTrajectory = swingCMPTrajectories.get(stepIndex);
         cmpSegments = swingCMPTrajectory.getNumberOfSegments();
         for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
         {
            cmpTrajectories.add(swingCMPTrajectory.getSegment(cmpSegment));
            totalNumberOfCMPSegments.increment();
         }
      }

      SegmentedFrameTrajectory3D transferCMPTrajectory = transferCMPTrajectories.get(numberOfSteps);
      cmpSegments = transferCMPTrajectory.getNumberOfSegments();
      for (int cmpSegment = 0; cmpSegment < cmpSegments; cmpSegment++)
      {
         cmpTrajectories.add(transferCMPTrajectory.getSegment(cmpSegment));
         totalNumberOfCMPSegments.increment();
      }

      numberOfSegmentsSwing0 = swingCMPTrajectories.get(0).getNumberOfSegments();

      initialize();
   }

   @Override
   public void initialize()
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
         comPositionDesiredInitialCurrentSegment.set(comPositionDesiredCurrent); // TODO; set to 1*dt after comPositionDesiredCurrent
         comVelocityDesiredInitialCurrentSegment.set(comVelocityDesiredCurrent);
         comAccelerationDesiredInitialCurrentSegment.set(comAccelerationDesiredCurrent);
      }

      comToolbox.computeDesiredCenterOfMassCornerData(icpDesiredFinalPositions, comDesiredInitialPositions,
                                                      comDesiredFinalPositions, comDesiredInitialVelocities, comDesiredFinalVelocities,
                                                      comDesiredInitialAccelerations, comDesiredFinalAccelerations, cmpTrajectories,
                                                      comPositionDesiredInitialCurrentSegment, comVelocityDesiredInitialCurrentSegment,
                                                      comAccelerationDesiredInitialCurrentSegment, omega0.getDoubleValue());
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
      comToolbox.computeDesiredCenterOfMassPosition(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalCurrentSegment,
                                                    comPositionDesiredInitialCurrentSegment, cmpPolynomial3D, comPositionDesiredCurrent);
      comToolbox.computeDesiredCenterOfMassVelocity(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(), icpPositionDesiredFinalCurrentSegment,
                                                    comPositionDesiredInitialCurrentSegment, cmpPolynomial3D, comVelocityDesiredCurrent);
      comToolbox.computeDesiredCenterOfMassAcceleration(omega0.getDoubleValue(), localTimeInCurrentPhase.getDoubleValue(),
                                                        icpPositionDesiredFinalCurrentSegment, comPositionDesiredInitialCurrentSegment, cmpPolynomial3D,
                                                        comAccelerationDesiredCurrent);
   }

   private int getCurrentSegmentIndex(double timeInCurrentPhase, List<FrameTrajectory3D> cmpTrajectories)
   {
      int currentSegmentIndex = FIRST_SEGMENT;
      while (timeInCurrentPhase > cmpTrajectories.get(currentSegmentIndex).getFinalTime()
            && Math.abs(cmpTrajectories.get(currentSegmentIndex).getFinalTime() - cmpTrajectories.get(currentSegmentIndex + 1).getInitialTime()) < 1.0e-5)
      {
         currentSegmentIndex++;
         if (currentSegmentIndex + 1 > cmpTrajectories.size())
         {
            return currentSegmentIndex;
         }
      }
      return currentSegmentIndex;
   }

   public void getICPPositionDesiredFinalFromSegment(FramePoint3D icpPositionDesiredFinal, int segment)
   {
      icpPositionDesiredFinal.set(icpDesiredFinalPositions.get(segment));
   }

   public void getPositionDesiredInitialFromSegment(FramePoint3D comPositionDesiredInitial, int segment)
   {
      comPositionDesiredInitial.set(comDesiredInitialPositions.get(segment));
   }

   public void getPositionDesiredFinalFromSegment(FramePoint3D comPositionDesiredFinal, int segment)
   {
      comPositionDesiredFinal.set(comDesiredFinalPositions.get(segment));
   }

   public void getFinalCoMPositionInSwing(FramePoint3D comPositionDesiredFinalToPack)
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

   public void getFinalCoMPositionInTransfer(FramePoint3D comPositionDesiredFinalToPack)
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

   public void getPosition(YoFramePoint positionToPack)
   {
      positionToPack.set(comPositionDesiredCurrent);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.set(comVelocityDesiredCurrent);
   }

   public void getVelocity(YoFrameVector velocityToPack)
   {
      velocityToPack.set(comVelocityDesiredCurrent);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.set(comAccelerationDesiredCurrent);
   }

   public void getAcceleration(YoFrameVector accelerationToPack)
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

   public void getLinearData(YoFramePoint positionToPack, YoFrameVector velocityToPack, YoFrameVector accelerationToPack)
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

   public List<FramePoint3D> getCoMPositionDesiredInitialList()
   {
      return comDesiredInitialPositions;
   }

   public List<FramePoint3D> getCoMPositionDesiredFinalList()
   {
      return comDesiredFinalPositions;
   }

   public List<FrameVector3D> getCoMVelocityDesiredInitialList()
   {
      return comDesiredInitialVelocities;
   }

   public List<FrameVector3D> getCoMVelocityDesiredFinalList()
   {
      return comDesiredFinalVelocities;
   }

   public List<FrameVector3D> getCoMAccelerationDesiredInitialList()
   {
      return comDesiredInitialAccelerations;
   }

   public List<FrameVector3D> getCoMAccelerationDesiredFinalList()
   {
      return comDesiredFinalAccelerations;
   }

   public int getTotalNumberOfSegments()
   {
      return totalNumberOfCMPSegments.getIntegerValue();
   }
}
