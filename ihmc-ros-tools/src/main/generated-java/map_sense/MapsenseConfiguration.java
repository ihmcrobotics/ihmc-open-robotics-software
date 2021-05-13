package map_sense;

import org.ros.internal.message.Message;
import std_msgs.Header;

public interface MapsenseConfiguration extends Message
{
   static final String _TYPE = "map_sense/MapsenseConfiguration";
   static final String _DEFINITION =
           "std_msgs/Header header\n"
         + "float32 mergeAngularThreshold\n"
         + "float32 mergeDistanceThreshold\n"
         + "uint8 filterSize\n"
         + "uint8 kernelLevel";

   Header getHeader();

   void setHeader(Header id);

   Float getMergeAngularThreshold();

   void setMergeAngularThreshold(Float thres);

   Float getMergeDistanceThreshold();

   void setMergeDistanceThreshold(Float thres);

   byte getFilterSize();

   void setFilterSize(byte numOfRegions);

   byte getKernelLevel();

   void setKernelLevel(byte numOfRegions);
}
