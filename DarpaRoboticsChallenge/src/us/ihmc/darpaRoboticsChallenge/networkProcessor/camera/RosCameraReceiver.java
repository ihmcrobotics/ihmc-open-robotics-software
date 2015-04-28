package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import geometry_msgs.Transform;

import java.awt.image.BufferedImage;
import java.net.URI;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.Time;

import sensor_msgs.CameraInfo;
import transform_provider.TransformProvider;
import transform_provider.TransformProviderRequest;
import transform_provider.TransformProviderResponse;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosCompressedImageSubscriber;
import boofcv.struct.calib.IntrinsicParameters;

public class RosCameraReceiver extends CameraDataReceiver
{
   private final IntrinsicParameters intrinsicParameters = new IntrinsicParameters();
   private static final boolean DEBUG = false;
   
   public RosCameraReceiver(SDFFullRobotModelFactory fullRobotModelFactory, final DRCRobotCameraParameters cameraParameters,
         RobotConfigurationDataBuffer robotConfigurationDataBuffer, final RosMainNode rosMainNode, final PacketCommunicator packetCommunicator,
         final PPSTimestampOffsetProvider ppsTimestampOffsetProvider, final CameraLogger logger, URI sensorURI)
   {
      super(fullRobotModelFactory, cameraParameters.getPoseFrameForSdf(), robotConfigurationDataBuffer, packetCommunicator, ppsTimestampOffsetProvider);
      if (cameraParameters.useRosForTransformFromPoseToSensor())
      {
         
         // Start request for transform
         ROSHeadTransformFrame cameraFrame = new ROSHeadTransformFrame(rosMainNode, cameraParameters);
         setCameraFrame(cameraFrame);
         new Thread(cameraFrame).start();
         
      }
      else
      {
         setCameraFrame(getHeadFrame());
      }

      if(cameraParameters.useIntrinsicParametersFromRos())
      {
         rosMainNode.attachSubscriber(cameraParameters.getRosCameraInfoTopicName(), new RosCameraInfoSubscriber(cameraParameters.getRosCameraInfoTopicName()));
      }
      else
      {
         throw new RuntimeException("You really want to use intrinisic parameters from ROS");
      }
      
      RosCompressedImageSubscriber imageSubscriberSubscriber = new RosCompressedImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            if (logger != null)
            {
               logger.log(image, timeStamp);
            }
            IntrinsicParameters intrinsicParameters;
            synchronized (RosCameraReceiver.this.intrinsicParameters)
            {
               intrinsicParameters = new IntrinsicParameters(RosCameraReceiver.this.intrinsicParameters);               
            }
            if(DEBUG)
            {
               PrintTools.debug(this, "Sending intrinsicParameters");
               intrinsicParameters.print();
            }
            updateLeftEyeImage(image, timeStamp, intrinsicParameters);

         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriberSubscriber);
   }

   private class ROSHeadTransformFrame extends ReferenceFrame implements Runnable
   {
      private static final long serialVersionUID = 6681193023636643459L;
      private final RosServiceClient<TransformProviderRequest, TransformProviderResponse> client;
      private final DRCRobotCameraParameters cameraParameters;
      
      private final RigidBodyTransform headToCameraTransform = new RigidBodyTransform();
      
      private ROSHeadTransformFrame(RosMainNode rosMainNode, DRCRobotCameraParameters cameraParameters)
      {
         super("rosHeadToCameraFrame", getHeadFrame(), true, false, false);
         this.cameraParameters = cameraParameters;
         this.client = new RosServiceClient<TransformProviderRequest, TransformProviderResponse>(TransformProvider._TYPE);
         rosMainNode.attachServiceClient("transform_provider", client);
      }

      public void run()
      {
         TransformProviderResponse response = null;
         client.waitTillConnected();
         while (response == null)
         {
            ThreadTools.sleep(1); // Don't hog CPU
            TransformProviderRequest request = client.getMessage();
            request.setTime(new Time(0));
            request.setSrc(cameraParameters.getBaseFrameForRosTransform());
            request.setDest(cameraParameters.getEndFrameForRosTransform());
            response = client.call(request);
         }
         Transform transform = response.getTransform().getTransform();
         Vector3d translation = new Vector3d(transform.getTranslation().getX(), transform.getTranslation().getY(), transform.getTranslation().getZ());
         Quat4d rotation = new Quat4d(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
         
         synchronized(headToCameraTransform)
         {
            headToCameraTransform.set(rotation, translation);
            System.out.println("Got head to camera transform");
            System.out.println(headToCameraTransform);
         }  
         
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         synchronized (headToCameraTransform)
         {
            transformToParent.set(headToCameraTransform);
         }
      }
   }
   
   private class RosCameraInfoSubscriber extends AbstractRosTopicSubscriber<CameraInfo> {

      public RosCameraInfoSubscriber( String which )
      {
         super(sensor_msgs.CameraInfo._TYPE);
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
           if(DEBUG)
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

}
