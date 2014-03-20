package us.ihmc.utilities.ros;

import org.ros.message.Time;
import sensor_msgs.JointState;
import std_msgs.Header;
import java.util.List;

public class RosJointStatePublisher extends RosTopicPublisher<JointState>
{
   private final JointState initialValue;
   private int seq = 0;
/*
   private NtpTimeProvider tp;
*/

   public RosJointStatePublisher(boolean latched)
   {
      this(latched, null);
/*
       InetAddress myIP = RosTools.getMyIP("http://localhost:11311");
       tp = new NtpTimeProvider(myIP);
*/
   }

   public RosJointStatePublisher(boolean latched, JointState initialValue)
   {
      super(JointState._TYPE, latched);
      this.initialValue = initialValue;
   }
   
   @Override
   public void connected()
   {
      if(initialValue != null)
      {
         publish(initialValue);
      }
   }


   public void publish(List<String> name, double[] position, double[] velocity, double[] effort, double t)
   {
      JointState message = getMessage();
      Header header = message.getHeader();

      header.setStamp(new Time(System.currentTimeMillis() / 1000.0 ));
      System.out.println("time: " + System.currentTimeMillis() / 1000.0);

      header.setFrameId("/world");
      header.setSeq(seq++);
      message.setHeader(header);

      if (name != null) {
         message.setName(name);
      }

      if (position != null) {
         message.setPosition(position);
      }

      if(velocity != null) {
        message.setVelocity(velocity);
      }

      if(effort != null) {
        message.setEffort(effort);
      }

      publish(message);
   }
}
