package us.ihmc.robotEnvironmentAwareness.ui.viewer;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

public class StereoFrameViewer extends AbstractSensorFrameViewer<StereoVisionPointCloudMessage>
{
   public StereoFrameViewer(REAUIMessager uiMessager, Topic<StereoVisionPointCloudMessage> messageState)
   {
      super(uiMessager, messageState);
   }

   @Override
   public void handleMessage(StereoVisionPointCloudMessage message)
   {
      if (message == null)
         return;
      Quaternion orientation = message.getSensorOrientation();
      Point3D position = message.getSensorPosition();
      lastAffine.set(JavaFXTools.createAffineFromQuaternionAndTuple(orientation, position));
   }
}
