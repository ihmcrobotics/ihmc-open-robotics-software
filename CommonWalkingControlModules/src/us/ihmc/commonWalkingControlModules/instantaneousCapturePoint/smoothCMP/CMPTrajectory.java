package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.YoFramePolynomial3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class CMPTrajectory
{
   private static final int maxNumberOfWaypoints = 50;

   private final YoInteger numberOfSegments;
   private final YoInteger trajectoryIndex;
   private final YoDouble timeIntoStep;

   private final YoFramePolynomial3D[] availablePolynomials = new YoFramePolynomial3D[maxNumberOfWaypoints];

   private final List<YoFramePolynomial3D> segments = new ArrayList<>();
   private boolean haveSetActiveSegments = false;

   private ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

   public CMPTrajectory(String namePrefix, YoVariableRegistry registry)
   {
      numberOfSegments = new YoInteger(namePrefix + "NumberOfCMPSegments", registry);
      trajectoryIndex = new YoInteger(namePrefix + "TrajectoryIndex", registry);
      timeIntoStep = new YoDouble(namePrefix + "TimeIntoStep", registry);

      for (int i = 0; i < maxNumberOfWaypoints; i++)
         availablePolynomials[i] = new YoFramePolynomial3D(namePrefix + i + "Polynomial", 5, referenceFrame, registry);
   }

   public void reset()
   {
      segments.clear();
      for (int i = 0; i < numberOfSegments.getIntegerValue(); i++)
         availablePolynomials[i].reset();

      numberOfSegments.set(0);
      trajectoryIndex.set(0);
      haveSetActiveSegments = false;
   }

   public YoFramePolynomial3D getNextSegment()
   {
      YoFramePolynomial3D activeSegment = availablePolynomials[numberOfSegments.getIntegerValue()];
      numberOfSegments.increment();

      return activeSegment;
   }

   private final FramePoint cmpPositionToThrowAway = new FramePoint();
   private final FrameVector cmpVelocityToThrowAway = new FrameVector();
   private final FrameVector cmpAccelerationToThrowAway = new FrameVector();

   public void update(double timeInState)
   {
      update(timeInState, cmpPositionToThrowAway);
   }

   public void update(double timeInState, FramePoint desiredCMPToPack)
   {
      update(timeInState, desiredCMPToPack, cmpVelocityToThrowAway);
   }

   public void update(double timeInState, FramePoint desiredCMPToPack, FrameVector desiredCMPVelocityToPack)
   {
      update(timeInState, desiredCMPToPack, desiredCMPVelocityToPack, cmpAccelerationToThrowAway);
   }

   public void update(double timeInState, FramePoint desiredCMPToPack, FrameVector desiredCMPVelocityToPack, FrameVector desiredCMPAccelerationToPack)
   {
      if (!availablePolynomials[trajectoryIndex.getIntegerValue()].timeIntervalContains(timeInState)
         && (trajectoryIndex.getIntegerValue() < numberOfSegments.getIntegerValue() -1))
      {
         trajectoryIndex.increment();
      }

      YoFramePolynomial3D currentPolynomial = availablePolynomials[trajectoryIndex.getIntegerValue()];
      currentPolynomial.compute(timeInState);
      currentPolynomial.getFramePosition(desiredCMPToPack);
      currentPolynomial.getFrameVelocity(desiredCMPVelocityToPack);
      currentPolynomial.getFrameAcceleration(desiredCMPAccelerationToPack);

      timeIntoStep.set(timeInState);
   }

   public boolean isDone()
   {
      boolean currentIsLast = trajectoryIndex.getIntegerValue() == numberOfSegments.getIntegerValue() - 1;
      boolean currentIsDone = !availablePolynomials[trajectoryIndex.getIntegerValue()].timeIntervalContains(timeIntoStep.getDoubleValue());

      return currentIsLast && currentIsDone;
   }

   public List<YoFramePolynomial3D> getPolynomials()
   {
      if (!haveSetActiveSegments)
      {
         for (int i = 0; i < numberOfSegments.getIntegerValue(); i++)
            segments.add(availablePolynomials[i]);

         haveSetActiveSegments = true;
      }

      return segments;
   }

   public int getNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }
}
