package map_sense;

import java.util.List;

public interface RawGPUPlanarRegionList extends org.ros.internal.message.Message
{
   static final String _TYPE = "map_sense/RawGPUPlanarRegionList";
   static final String _DEFINITION = "std_msgs/Header header\n" + "map_sense/RawGPUPlanarRegion[] regions\n" + "uint16 numOfRegions";

   std_msgs.Header getHeader();

   void setHeader(std_msgs.Header id);

   Short getNumOfRegions();

   void setNumOfRegions(Short numOfRegions);

   List<RawGPUPlanarRegion> getRegions();

   void setRegions(List<map_sense.RawGPUPlanarRegion> value);
}
