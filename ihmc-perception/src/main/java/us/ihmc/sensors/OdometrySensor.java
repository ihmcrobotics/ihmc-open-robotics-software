package us.ihmc.sensors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;

public interface OdometrySensor
{
   String getSensorName();

   Quaternion getImuOrientation();

   ReferenceFrame getTrackedSensorFrame();

   long getLastGrabTimestamp();
}
