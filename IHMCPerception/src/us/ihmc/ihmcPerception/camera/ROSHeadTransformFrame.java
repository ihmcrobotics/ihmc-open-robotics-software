package us.ihmc.ihmcPerception.camera;

import geometry_msgs.Transform;
import org.ros.message.Time;
import transform_provider.TransformProvider;
import transform_provider.TransformProviderRequest;
import transform_provider.TransformProviderResponse;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorParameters;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosServiceClient;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

public class ROSHeadTransformFrame extends ReferenceFrame implements Runnable
{
   private static final long serialVersionUID = 6681193023636643459L;
   private final RosServiceClient<TransformProviderRequest, TransformProviderResponse> client;
   private final DRCRobotSensorParameters cameraParameters;

   private final RigidBodyTransform headToCameraTransform = new RigidBodyTransform();

   public ROSHeadTransformFrame(ReferenceFrame headFrame, RosMainNode rosMainNode, DRCRobotSensorParameters cameraParameters)
   {
      super("rosHeadToCameraFrame", headFrame, true, false, false);
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
      Quat4d rotation = new Quat4d(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation()
            .getW());

      synchronized (headToCameraTransform)
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
