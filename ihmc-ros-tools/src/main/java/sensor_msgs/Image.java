package sensor_msgs;

//import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import std_msgs.Header;

public interface Image extends Message {
   String _TYPE = "sensor_msgs/Image";
   String _DEFINITION = "# This message contains an uncompressed image\n# (0, 0) is at top-left corner of image\n#\n\nHeader header        # Header timestamp should be acquisition time of image\n                     # Header frame_id should be optical frame of camera\n                     # origin of frame should be optical center of cameara\n                     # +x should point to the right in the image\n                     # +y should point down in the image\n                     # +z should point into to plane of the image\n                     # If the frame_id here and the frame_id of the CameraInfo\n                     # message associated with the image conflict\n                     # the behavior is undefined\n\nuint32 height         # image height, that is, number of rows\nuint32 width          # image width, that is, number of columns\n\n# The legal values for encoding are in file src/image_encodings.cpp\n# If you want to standardize a new string format, join\n# ros-users@lists.sourceforge.net and send an email proposing a new encoding.\n\nstring encoding       # Encoding of pixels -- channel meaning, ordering, size\n                      # taken from the list of strings in include/sensor_msgs/image_encodings.h\n\nuint8 is_bigendian    # is this data bigendian?\nuint32 step           # Full row length in bytes\nuint8[] data          # actual matrix data, size is (step * rows)\n";

   Header getHeader();

   void setHeader(Header var1);

   int getHeight();

   void setHeight(int var1);

   int getWidth();

   void setWidth(int var1);

   String getEncoding();

   void setEncoding(String var1);

   byte getIsBigendian();

   void setIsBigendian(byte var1);

   int getStep();

   void setStep(int var1);

//   ChannelBuffer getData();
//
//   void setData(ChannelBuffer var1);
}