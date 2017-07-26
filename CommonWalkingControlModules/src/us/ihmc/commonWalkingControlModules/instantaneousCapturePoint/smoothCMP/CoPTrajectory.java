package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.configurations.CoPSplineType;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CoPTrajectory implements CoPTrajectoryInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<YoFrameTrajectory3D> segments = new ArrayList<>();

   private final YoInteger numberOfSegments;
   private final YoInteger currentSegmentIndex;
   private final YoDouble currentTrajectoryTime;
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
      currentTrajectoryTime = new YoDouble(namePrefix + stepNumber + type.toString() + "CurrentTrajectoryTime", registry);
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
      currentSegmentIndex.set(0);
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
      currentTrajectoryTime.set(timeInState);
      setCurrentSegmentIndexFromStateTime(timeInState);
      currentSegment.compute(timeInState);
      currentSegment.getFramePosition(desiredCoPToPack);
      currentSegment.getFrameVelocity(desiredCoPVelocityToPack);
      currentSegment.getFrameAcceleration(desiredCoPAccelerationToPack);
   }

   private void setCurrentSegmentIndexFromStateTime(double timeInState)
   {
      if (timeInState < 0)
         throw new RuntimeException(name + ": Must actually be in the state: " + timeInState);

      while (!segments.get(currentSegmentIndex.getIntegerValue()).timeIntervalContains(timeInState)
            && currentSegmentIndex.getIntegerValue() < numberOfSegments.getIntegerValue())
      {
         currentSegmentIndex.increment();
      }
      currentSegment = segments.get(currentSegmentIndex.getIntegerValue());

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
      YoFrameTrajectory3D segment = segments.get(numberOfSegments.getIntegerValue());

      switch (segmentOrder)
      {
      case CUBIC:
         segment.setCubic(initialTime, finalTime, initialPosition, finalPosition);
         break;
      default:
         segment.setLinear(initialTime, finalTime, initialPosition, finalPosition);
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
