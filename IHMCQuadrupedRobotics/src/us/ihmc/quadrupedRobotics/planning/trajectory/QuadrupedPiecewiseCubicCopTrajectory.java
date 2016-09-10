package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import java.util.ArrayList;

public class QuadrupedPiecewiseCubicCopTrajectory
{
   YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable trajectoryInitialized;
   private final IntegerYoVariable numberOfIntervals;
   private final YoFramePoint copPositionAtCurrentTime;
   private final YoFrameVector copVelocityAtCurrentTime;
   private final ArrayList<TimeInterval> copTrajectoryTimeIntervals;
   private final ArrayList<YoPolynomial[]> copTrajectoryPolynominals;

   public QuadrupedPiecewiseCubicCopTrajectory(int maximumNumberOfContactPhases, YoVariableRegistry parentRegistry)
   {
      int maximumNumberOfTrajectorySegments = 3 * maximumNumberOfContactPhases;
      trajectoryInitialized = new BooleanYoVariable("copTrajectoryInitialized", registry);
      numberOfIntervals = new IntegerYoVariable("copNumberOfIntervals", registry);
      numberOfIntervals.set(0);
      copPositionAtCurrentTime = new YoFramePoint("copPositionAtCurrentTime", ReferenceFrame.getWorldFrame(), registry);
      copVelocityAtCurrentTime = new YoFrameVector("copVelocityAtCurrentTime", ReferenceFrame.getWorldFrame(), registry);
      copTrajectoryTimeIntervals = new ArrayList<>(maximumNumberOfTrajectorySegments);
      copTrajectoryPolynominals = new ArrayList<>(maximumNumberOfTrajectorySegments);
      for (int i = 0; i < maximumNumberOfTrajectorySegments; i++)
      {
         copTrajectoryTimeIntervals.set(i, new TimeInterval());
         copTrajectoryPolynominals.set(i, new YoPolynomial[3]);
         copTrajectoryPolynominals.get(i)[0] = new YoPolynomial("copTrajectoryPolynomial" + i + "X", 4, registry);
         copTrajectoryPolynominals.get(i)[1] = new YoPolynomial("copTrajectoryPolynomial" + i + "Y", 4, registry);
         copTrajectoryPolynominals.get(i)[2] = new YoPolynomial("copTrajectoryPolynomial" + i + "Z", 4, registry);
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void initializeTrajectory(QuadrupedTimedContactSequence contactSequence)
   {
   }

   public void computeTrajectory(double currentTime)
   {
      if (!trajectoryInitialized.getBooleanValue())
      {
         throw new RuntimeException("trajectory must be initialized before calling computeTrajectory");
      }

      double startTime = copTrajectoryTimeIntervals.get(0).getStartTime();
      double endTime = copTrajectoryTimeIntervals.get(numberOfIntervals.getIntegerValue() - 1).getEndTime();
      currentTime = MathTools.clipToMinMax(currentTime, startTime, endTime);

      for (int i = 0; i < numberOfIntervals.getIntegerValue(); i++)
      {
         TimeInterval timeInterval = copTrajectoryTimeIntervals.get(i);
         YoPolynomial[] polynomial = copTrajectoryPolynominals.get(i);
         if (currentTime <= timeInterval.getEndTime())
         {
            for (int j = 0; j < 3; j++)
            {
               polynomial[j].compute(currentTime);
            }
            copPositionAtCurrentTime.set(polynomial[0].getPosition(), polynomial[1].getPosition(), polynomial[2].getPosition());
            copPositionAtCurrentTime.set(polynomial[0].getVelocity(), polynomial[1].getVelocity(), polynomial[2].getVelocity());
            return;
         }
      }
   }

   public void getPosition(FramePoint copPositionAtCurrentTime)
   {
      copPositionAtCurrentTime.setIncludingFrame(this.copPositionAtCurrentTime.getFrameTuple());
   }

   public void getVelocity(FrameVector copVelocityAtCurrentTime)
   {
      copVelocityAtCurrentTime.setIncludingFrame(this.copVelocityAtCurrentTime.getFrameTuple());
   }
}
