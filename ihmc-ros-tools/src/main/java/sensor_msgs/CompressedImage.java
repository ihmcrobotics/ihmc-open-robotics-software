package sensor_msgs;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface CompressedImage extends Message {
   String _TYPE = "sensor_msgs/CompressedImage";
   String _DEFINITION = "# This message contains a compressed image\n\nHeader header        # Header timestamp should be acquisition time of image\n                     # Header frame_id should be optical frame of camera\n                     # origin of frame should be optical center of cameara\n                     # +x should point to the right in the image\n                     # +y should point down in the image\n                     # +z should point into to plane of the image\n\nstring format        # Specifies the format of the data\n                     #   Acceptable values:\n                     #     jpeg, png\nuint8[] data         # Compressed image buffer\n";

   Header getHeader();

   void setHeader(Header var1);

   String getFormat();

   void setFormat(String var1);

//   ChannelBuffer getData();
//
//   void setData(ChannelBuffer var1);
}