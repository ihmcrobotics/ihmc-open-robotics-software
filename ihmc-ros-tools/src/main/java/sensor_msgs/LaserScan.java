package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface LaserScan extends Message {
   String _TYPE = "sensor_msgs/LaserScan";
   String _DEFINITION = "# Single scan from a planar laser range-finder\n#\n# If you have another ranging device with different behavior (e.g. a sonar\n# array), please find or create a different message, since applications\n# will make fairly laser-specific assumptions about this data\n\nHeader header            # timestamp in the header is the acquisition time of \n                         # the first ray in the scan.\n                         #\n                         # in frame frame_id, angles are measured around \n                         # the positive Z axis (counterclockwise, if Z is up)\n                         # with zero angle being forward along the x axis\n                         \nfloat32 angle_min        # start angle of the scan [rad]\nfloat32 angle_max        # end angle of the scan [rad]\nfloat32 angle_increment  # angular distance between measurements [rad]\n\nfloat32 time_increment   # time between measurements [seconds] - if your scanner\n                         # is moving, this will be used in interpolating position\n                         # of 3d points\nfloat32 scan_time        # time between scans [seconds]\n\nfloat32 range_min        # minimum range value [m]\nfloat32 range_max        # maximum range value [m]\n\nfloat32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)\nfloat32[] intensities    # intensity data [device-specific units].  If your\n                         # device does not provide intensities, please leave\n                         # the array empty.\n";

   Header getHeader();

   void setHeader(Header var1);

   float getAngleMin();

   void setAngleMin(float var1);

   float getAngleMax();

   void setAngleMax(float var1);

   float getAngleIncrement();

   void setAngleIncrement(float var1);

   float getTimeIncrement();

   void setTimeIncrement(float var1);

   float getScanTime();

   void setScanTime(float var1);

   float getRangeMin();

   void setRangeMin(float var1);

   float getRangeMax();

   void setRangeMax(float var1);

   float[] getRanges();

   void setRanges(float[] var1);

   float[] getIntensities();

   void setIntensities(float[] var1);
}