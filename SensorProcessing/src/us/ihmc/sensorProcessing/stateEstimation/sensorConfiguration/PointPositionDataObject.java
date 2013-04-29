package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class PointPositionDataObject
{
   protected final FramePoint measurementPointInBodyFrame = new FramePoint();
   protected final FramePoint positionOfMeasurementPointInWorldFrame = new FramePoint(ReferenceFrame.getWorldFrame());

   public void set(FramePoint measurementPointInBodyFrame, FramePoint positionOfMeasurementPointInWorldFrame)
   {
      this.measurementPointInBodyFrame.setAndChangeFrame(measurementPointInBodyFrame);
      this.positionOfMeasurementPointInWorldFrame.set(positionOfMeasurementPointInWorldFrame);
   }

   public FramePoint getMeasurementPointInWorldFrame()
   {
      return positionOfMeasurementPointInWorldFrame;
   }

   public FramePoint getMeasurementPointInBodyFrame()
   {
      return measurementPointInBodyFrame;
   }

   public void set(PointPositionDataObject other)
   {
      set(other.getMeasurementPointInBodyFrame(), other.getMeasurementPointInWorldFrame());
   }

   public boolean epsilonEquals(PointPositionDataObject other, double epsilon)
   {
      if (getMeasurementPointInBodyFrame().getReferenceFrame() != other.getMeasurementPointInBodyFrame().getReferenceFrame())
         return false;

      boolean bodyPointsEqual = getMeasurementPointInBodyFrame().epsilonEquals(other.getMeasurementPointInBodyFrame(), epsilon);
      boolean worldPointsEqual = getMeasurementPointInWorldFrame().epsilonEquals(other.getMeasurementPointInWorldFrame(), epsilon);
      return bodyPointsEqual && worldPointsEqual;
   }
}
