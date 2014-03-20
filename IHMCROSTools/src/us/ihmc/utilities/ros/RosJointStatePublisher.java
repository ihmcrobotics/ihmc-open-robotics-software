package us.ihmc.utilities.ros;

import org.ros.message.Time;
import org.ros.time.NtpTimeProvider;
import org.ros.time.TimeProvider;
import sensor_msgs.JointState;
import std_msgs.Header;

import java.net.InetAddress;
import java.net.URI;
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
      message.setName(name);
      message.setPosition(position);
      message.setVelocity(velocity);
      message.setEffort(effort);
      publish(message);
   }
}
