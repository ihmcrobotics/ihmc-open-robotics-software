package map_sense;

import geometry_msgs.Point;
import geometry_msgs.Vector3;
import org.ros.internal.message.Message;
import std_msgs.Header;

import java.util.List;

public interface MapsenseConfiguration extends Message
{
   static final String _TYPE = "map_sense/MapsenseConfiguration";
   static final String _DEFINITION =
         "std_msgs/Header header\n" + "float32 mergeAngularThreshold\n" + "float32 mergeDistanceThreshold\n" + "uint16 filterSize\n" + "uint16 kernelLevel";

   Header getHeader();

   void setHeader(Header id);

   Float getMergeAngularThreshold();

   void setMergeAngularThreshold(Float thres);

   Float getMergeDistanceThreshold();

   void setMergeDistanceThreshold(Float thres);

   Short getFilterSize();

   void setFilterSize(Short numOfRegions);

   Short getKernelLevel();

   void setKernelLevel(Short numOfRegions);
}
