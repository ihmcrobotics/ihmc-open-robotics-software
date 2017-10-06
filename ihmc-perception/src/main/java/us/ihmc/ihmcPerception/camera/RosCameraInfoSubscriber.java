package us.ihmc.ihmcPerception.camera;

import boofcv.struct.calib.IntrinsicParameters;
import sensor_msgs.CameraInfo;
import us.ihmc.commons.PrintTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class RosCameraInfoSubscriber extends AbstractRosTopicSubscriber<CameraInfo>
{
   private final IntrinsicParameters intrinsicParameters = new IntrinsicParameters();

   public RosCameraInfoSubscriber(String which)
   {
      super(sensor_msgs.CameraInfo._TYPE);
   }

   public IntrinsicParameters getIntrinisicParameters()
   {
      synchronized (intrinsicParameters)
      {
         return new IntrinsicParameters(intrinsicParameters);
      }

   }

   @Override
   public void onNewMessage(CameraInfo message)
   {
      double[] P = message.getP();

      // assume P is a 4x3 matrix which was not written by a CV person

      synchronized (intrinsicParameters)
      {
         intrinsicParameters.fx = P[0];
         intrinsicParameters.skew = P[1];
         intrinsicParameters.cx = P[2];
         intrinsicParameters.fy = P[5];
         intrinsicParameters.cy = P[6];

         // Use these parameters for Gazebo in the future.

         //            double[] K = message.getK();
         //
         //            // assume K is a 3x3 matrix in row major format and is in the standard format
         //            packet.param.fx = K[0];
         //            packet.param.skew = K[1];
         //            packet.param.cx = K[2];
         //            packet.param.fy = K[4];
         //            packet.param.cy = K[5];

         intrinsicParameters.width = message.getWidth();
         intrinsicParameters.height = message.getHeight();
         if (RosCameraReceiver.DEBUG)
         {
            PrintTools.debug(this, "Receiving intrinsicParameters");
            intrinsicParameters.print();
         }
         //calc field of view
         //            double hfov;
         //
         //            double theta0 = Math.atan(packet.param.cx / packet.param.fx);
         //            double theta1 = Math.atan((packet.param.width - packet.param.cx) / packet.param.fx);
         //            hfov = theta0 + theta1;
      }

   }
}