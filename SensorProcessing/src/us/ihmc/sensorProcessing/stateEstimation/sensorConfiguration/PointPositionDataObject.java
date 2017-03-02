package us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GenericCRC32;

public class PointPositionDataObject
{
   protected String bodyFixedReferenceFrameName;
   protected boolean isPointPositionValid = true;
   protected final Point3D measurementPointInBodyFrame = new Point3D();
   protected final Point3D positionOfMeasurementPointInWorldFrame = new Point3D();

   public void set(FramePoint measurementPointInBodyFrame, FramePoint positionOfMeasurementPointInWorldFrame, boolean isPointPositionValid)
   {
      bodyFixedReferenceFrameName = measurementPointInBodyFrame.getReferenceFrame().getName();
      this.isPointPositionValid = isPointPositionValid;
      positionOfMeasurementPointInWorldFrame.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      measurementPointInBodyFrame.get(this.measurementPointInBodyFrame);
      positionOfMeasurementPointInWorldFrame.get(this.positionOfMeasurementPointInWorldFrame);
   }

   public Point3D getMeasurementPointInWorldFrame()
   {
      return positionOfMeasurementPointInWorldFrame;
   }

   public Point3D getMeasurementPointInBodyFrame()
   {
      return measurementPointInBodyFrame;
   }

   public void set(PointPositionDataObject other)
   {  
      bodyFixedReferenceFrameName = other.bodyFixedReferenceFrameName;
      isPointPositionValid = other.isPointPositionValid;
      measurementPointInBodyFrame.set(other.measurementPointInBodyFrame);
      positionOfMeasurementPointInWorldFrame.set(other.positionOfMeasurementPointInWorldFrame);
   }

   public boolean epsilonEquals(PointPositionDataObject other, double epsilon)
   {
      if (bodyFixedReferenceFrameName != other.bodyFixedReferenceFrameName)
         return false;

      boolean validStateEqual = isPointPositionValid == other.isPointPositionValid;
      boolean bodyPointsEqual = getMeasurementPointInBodyFrame().epsilonEquals(other.getMeasurementPointInBodyFrame(), epsilon);
      boolean worldPointsEqual = getMeasurementPointInWorldFrame().epsilonEquals(other.getMeasurementPointInWorldFrame(), epsilon);
      return validStateEqual && bodyPointsEqual && worldPointsEqual;
   }

   public String getBodyFixedReferenceFrameName()
   {
      return bodyFixedReferenceFrameName;
   }

   public boolean isPointPositionValid()
   {
      return isPointPositionValid;
   }

   public void invalidatePointPosition()
   {
      isPointPositionValid = false;
   }

   public void calculateChecksum(GenericCRC32 checksum)
   {
      checksum.update(isPointPositionValid);
      checksum.update(bodyFixedReferenceFrameName);
      checksum.update(measurementPointInBodyFrame);
      checksum.update(positionOfMeasurementPointInWorldFrame);
   }
}
