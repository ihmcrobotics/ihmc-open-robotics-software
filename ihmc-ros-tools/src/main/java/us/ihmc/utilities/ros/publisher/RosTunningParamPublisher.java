package us.ihmc.utilities.ros.publisher;

import lidar_obstacle_detection.TunningParam;

public class RosTunningParamPublisher extends RosTopicPublisher<TunningParam>
{
   public RosTunningParamPublisher()
   {
      super(TunningParam._TYPE, false);
   }

   public void publish(double tunningParam1, double tunningParam2, double tunningParam3, double tunningParam4
   ,double tunningParam5, double tunningParam6, double tunningParam7, double tunningParam8, double tunningParam9)
   {
      TunningParam message = getMessage();
      message.setTunningParam1(tunningParam1);
      message.setTunningParam2(tunningParam2);
      message.setTunningParam3(tunningParam3);
      message.setTunningParam4(tunningParam4);
      message.setTunningParam5(tunningParam5);
      message.setTunningParam6(tunningParam6);
      message.setTunningParam7(tunningParam7);
      message.setTunningParam8(tunningParam8);
      message.setTunningParam9(tunningParam9);
      publish(message);
   }
}
