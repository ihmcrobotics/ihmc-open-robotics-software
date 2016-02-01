package us.ihmc.utilities.ros.publisher;

import java.util.ArrayList;
import java.util.List;

import org.ros.message.Time;

import sensor_msgs.JointState;
import std_msgs.Header;

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


   public void publish(List<String> name, double[] position, double[] velocity, double[] effort, Time t)
   {
      JointState message = getMessage();

      Header header = message.getHeader();
      header.setStamp(t);
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
   
   public double[] toDoubleArray(float[] data)
   {
      double[] doubleData = new double[data.length];
      for(int i = 0; i < data.length; i++)
      {
         doubleData[i] = data[i];
      }
      return doubleData;
   }

   public void publish(ArrayList<String> nameList, float[] jointAngles, float[] jointVelocities, float[] jointTorques, Time t)
   {
      publish(nameList, toDoubleArray(jointAngles), toDoubleArray(jointVelocities), toDoubleArray(jointTorques), t);
   }
}
