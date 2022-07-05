package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

import static us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

public class MultipleCoMSegmentTrajectoryGenerator extends MultipleSegmentPositionTrajectoryGenerator<CoMTrajectorySegment>
{
   public MultipleCoMSegmentTrajectoryGenerator(String namePrefix, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, parentRegistry);
   }

   public MultipleCoMSegmentTrajectoryGenerator(String namePrefix,
                                                int maximumNumberOfWaypoints,
                                                YoRegistry parentRegistry)
   {
      super(namePrefix, maximumNumberOfWaypoints, ReferenceFrame.getWorldFrame(), CoMTrajectorySegment::new, parentRegistry);
   }

   public void appendLinearSegment(FramePoint3DReadOnly startPosition, FramePoint3DReadOnly endPosition, double omega, double startTime, double endTime)
   {
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + 1);
      CoMTrajectorySegment segment = segments.add();
      checkReferenceFrameMatch(segment);

      segment.getTimeInterval().setInterval(startTime, endTime);
      checkNextSegmentIsContinuous(segment);

      segment.setOmega(omega);

      double duration = endTime - startTime;
      double rateX = (endPosition.getX() - startPosition.getX()) / duration;
      double rateY = (endPosition.getY() - startPosition.getY()) / duration;
      double rateZ = (endPosition.getZ() - startPosition.getZ()) / duration;

      segment.setFirstCoefficient(0.0, 0.0, 0.0);
      segment.setSecondCoefficient(0.0, 0.0, 0.0);
      segment.setThirdCoefficient(0.0, 0.0, 0.0);
      segment.setFourthCoefficient(0.0, 0.0, 0.0);
      segment.setFifthCoefficient(rateX, rateY, rateZ);
      segment.setSixthCoefficient(startPosition.getX(), startPosition.getY(), startPosition.getZ());

      numberOfSegments.increment();
   }

   public void appendSegment(TimeIntervalReadOnly timeInterval, double omega, DMatrixRMaj coefficientsArray, int startRow)
   {
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + 1);
      CoMTrajectorySegment segment = segments.add();
      checkReferenceFrameMatch(segment);

      segment.getTimeInterval().set(timeInterval);
      checkNextSegmentIsContinuous(segment);

      segment.setOmega(omega);
      segment.setCoefficients(coefficientsArray, startRow);

      numberOfSegments.increment();
   }
}
