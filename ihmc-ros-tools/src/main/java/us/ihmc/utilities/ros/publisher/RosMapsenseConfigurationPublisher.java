package us.ihmc.utilities.ros.publisher;

import map_sense.MapsenseConfiguration;

public class RosMapsenseConfigurationPublisher extends RosTopicPublisher<MapsenseConfiguration>
{
   public RosMapsenseConfigurationPublisher()
   {
      super(MapsenseConfiguration._TYPE, false);
   }

   public void publish(byte kernelLevel, byte filterSize, float mergeAngular, float mergeDistance, float regionGrowthFactor, byte gaussianSize, byte gaussianSigma)
   {
      MapsenseConfiguration message = getMessage();
      message.setKernelLevel(kernelLevel);
      message.setFilterSize(filterSize);
      message.setMergeAngularThreshold(mergeAngular);
      message.setMergeDistanceThreshold(mergeDistance);
      message.setRegionGrowthFactor(regionGrowthFactor);
      message.setGaussianSize(gaussianSize);
      message.setGaussianSigma(gaussianSigma);
      publish(message);
   }
}
