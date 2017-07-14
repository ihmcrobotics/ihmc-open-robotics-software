package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class CoPTrajectory implements CoPTrajectoryInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<YoFrameTrajectory3D> segments = new ArrayList<>();

   private final YoInteger numberOfSegments;
   private final YoInteger currentSegmentIndex;
   private final CoPSplineType splineType;
   private final WalkingTrajectoryType trajectoryType;
   private final int stepNumber;
   
   private YoFrameTrajectory3D currentSegment;
   private final FramePoint copToThrowAway = new FramePoint();
   private final FrameVector copVelocityToThrowAway = new FrameVector();
   private final FrameVector copAccelerationToThrowAway = new FrameVector();
   private final String name;

   public CoPTrajectory(String namePrefix, int stepNumber, CoPSplineType splineType, int maxNumberOfSegments, WalkingTrajectoryType type,
                        YoVariableRegistry registry)
   {
      name = namePrefix + stepNumber + type.toString();
      numberOfSegments = new YoInteger(namePrefix + stepNumber + type.toString() + "NumberOfSegments", registry);
      currentSegmentIndex = new YoInteger(namePrefix + stepNumber + type.toString() + "CurrentSegment", registry);
      for (int i = 0; i < maxNumberOfSegments; i++)
      {
         YoFrameTrajectory3D segmentTrajectory = new YoFrameTrajectory3D(type.toString() + "Trajectory" + stepNumber + "Segment" + i,
                                                                         splineType.getNumberOfCoefficients(), worldFrame, registry);
         segments.add(segmentTrajectory);
      }
      this.splineType = splineType;
      this.trajectoryType = type;
      this.stepNumber = stepNumber;
      reset();
   }

   @Override
   public void reset()
   {
      for (int i = 0; i < segments.size(); i++)
         segments.get(i).reset();
      currentSegmentIndex.set(-1);
      currentSegment = null;
      numberOfSegments.set(0);
   }

   @Override
   public void update(double timeInState)
   {
      update(timeInState, copToThrowAway, copVelocityToThrowAway, copAccelerationToThrowAway);
   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack)
   {
      update(timeInState, desiredCoPToPack, copVelocityToThrowAway, copAccelerationToThrowAway);
   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack)
   {
      update(timeInState, desiredCoPToPack, desiredCoPVelocityToPack, copAccelerationToThrowAway);
   }

   @Override
   public void update(double timeInState, FramePoint desiredCoPToPack, FrameVector desiredCoPVelocityToPack, FrameVector desiredCoPAccelerationToPack)
   {
      setCurrentSegmentIndexFromStateTime(timeInState);
      currentSegment.compute(timeInState);
      currentSegment.getFramePosition(desiredCoPToPack);
      currentSegment.getFrameVelocity(desiredCoPVelocityToPack);
      currentSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
   }

   private void setCurrentSegmentIndexFromStateTime(double timeInState)
   {
      int segmentIndex = 0;
      for (; segmentIndex < segments.size(); segmentIndex++)
         if (segments.get(segmentIndex).timeIntervalContains(timeInState))
            break;
      if (segmentIndex == segments.size())
         throw new RuntimeException(name + ": Unable to locate suitable segment for given time:" + timeInState);
      currentSegment = segments.get(segmentIndex);
      currentSegmentIndex.set(segmentIndex);
   }

   @Override
   public List<YoFrameTrajectory3D> getPolynomials()
   {
      return segments;
   }

   @Override
   public int getNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }

   @Override
   public void setSegment(CoPSplineType segmentOrder, double initialTime, double finalTime, FramePoint initialPosition, FramePoint finalPosition)
   {
      PrintTools.debug("Step:" + stepNumber + " " + trajectoryType.toString() + " Trajectory " + numberOfSegments.getIntegerValue() + " , InitialTime: " + initialTime + " FinalTime: " + finalTime + " InitialPosition: "
            + initialPosition.toString() + " FinalPosition: " + finalPosition.toString());
      switch (segmentOrder)
      {
      case CUBIC:
         segments.get(numberOfSegments.getIntegerValue()).setCubic(initialTime, finalTime, initialPosition, finalPosition);
         break;

      default:
         segments.get(numberOfSegments.getIntegerValue()).setLinear(initialTime, finalTime, initialPosition, finalPosition);
         break;
      }
      numberOfSegments.increment();
   }

   public YoFrameTrajectory3D getCurrentSegment(double timeInState)
   {
      setCurrentSegmentIndexFromStateTime(timeInState);
      return currentSegment;
   }
}
