package us.ihmc.utilities.ros.publisher;

import map_sense.MapsenseConfiguration;

public class RosMapsenseConfigurationPublisher extends RosTopicPublisher<MapsenseConfiguration>
{

   public RosMapsenseConfigurationPublisher()
   {
      super(MapsenseConfiguration._TYPE, false);
   }

   public void publish(short kernelLevel, short filterSize, float mergeAngular, float mergeDistance)
   {
      MapsenseConfiguration message = getMessage();
      message.setKernelLevel(kernelLevel);
      message.setFilterSize(filterSize);
      message.setMergeAngularThreshold(mergeAngular);
      message.setMergeDistanceThreshold(mergeDistance);
      publish(message);
   }

}
