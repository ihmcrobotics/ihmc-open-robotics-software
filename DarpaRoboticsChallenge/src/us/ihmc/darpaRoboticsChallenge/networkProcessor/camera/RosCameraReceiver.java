package us.ihmc.darpaRoboticsChallenge.networkProcessor.camera;

import geometry_msgs.Transform;

import java.awt.image.BufferedImage;
import java.net.URI;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.Time;

import transform_provider.TransformProvider;
import transform_provider.TransformProviderRequest;
import transform_provider.TransformProviderResponse;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.communication.util.DRCSensorParameters;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;
import us.ihmc.utilities.ros.subscriber.RosImageSubscriber;

public class RosCameraReceiver extends CameraDataReceiver
{
   
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

      RosImageSubscriber imageSubscriberSubscriber = new RosImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            if (logger != null)
            {
               logger.log(image, timeStamp);
            }

            updateLeftEyeImage(image, timeStamp, DRCSensorParameters.DUMMY_FIELD_OF_VIEW);

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

}
