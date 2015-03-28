package us.ihmc.utilities.ros.subscriber;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import sensor_msgs.CameraInfo;
import us.ihmc.utilities.ros.RosMainNode;
import boofcv.struct.calib.IntrinsicParameters;

public class RosCameraInfoSubscriber extends AbstractRosTopicSubscriber<CameraInfo> {

   boolean isCameraInfoReceived=false;
   IntrinsicParameters param = new IntrinsicParameters();
   RosMainNode rosMainNode;
   CountDownLatch latch = new CountDownLatch(1);
   
   public RosCameraInfoSubscriber(RosMainNode rosMainNode, String cameraInfoTopic)
   {
      super(sensor_msgs.CameraInfo._TYPE);
      this.rosMainNode = rosMainNode;
      rosMainNode.attachSubscriber(cameraInfoTopic, this);
   }


   public IntrinsicParameters getIntrinsicParametersBlocking() throws InterruptedException
   {
      latch.await(5, TimeUnit.SECONDS);
      return param;
   }
   

   @Override
   public void onNewMessage(CameraInfo message)
   {
      // hack around ROS flooding it with these packets
      if( isCameraInfoReceived )
         return;
      isCameraInfoReceived = true;

      
      double[] P = message.getP();

      // assume P is a 4x3 matrix which was not written by a CV person
      param.fx = P[0];
      param.skew = P[1];
      param.cx = P[2];
      param.fy = P[5];
      param.cy = P[6];
      
      // Use these parameters for Gazebo in the future.

      //            double[] K = message.getK();
      //
      //            // assume K is a 3x3 matrix in row major format and is in the standard format
      //            packet.param.fx = K[0];
      //            packet.param.skew = K[1];
      //            packet.param.cx = K[2];
      //            packet.param.fy = K[4];
      //            packet.param.cy = K[5];

      param.width = message.getWidth();
      param.height = message.getHeight();
      //calc field of view
//      double hfov;
//
//      double theta0 = Math.atan(packet.param.cx / packet.param.fx);
//      double theta1 = Math.atan((packet.param.width - packet.param.cx) / packet.param.fx);
//      hfov = theta0 + theta1;


      rosMainNode.removeSubscriber(this);
      latch.countDown();
   }
}