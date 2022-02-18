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
         + "uint8 kernelLevel\n"
         + "uint8 gaussianSize\n"
         + "uint8 gaussianSigma\n"
         + "float32 regionGrowthFactor";

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

   byte getGaussianSize();

   void setGaussianSize(byte gaussianSize);

   byte getGaussianSigma();

   void setGaussianSigma(byte gaussianSigma);

   Float getRegionGrowthFactor();

   void setRegionGrowthFactor(Float growthFactor);
}
