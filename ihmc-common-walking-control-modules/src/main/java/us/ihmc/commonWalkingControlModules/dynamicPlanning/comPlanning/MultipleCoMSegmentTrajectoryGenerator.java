package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;
import java.util.function.Supplier;

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

      segment.setFifthCoefficient(ReferenceFrame.getWorldFrame(), rateX, rateY, rateZ);
      segment.setSixthCoefficient(ReferenceFrame.getWorldFrame(), startPosition.getX(), startPosition.getY(), startPosition.getZ());

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
