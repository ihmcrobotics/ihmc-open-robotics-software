package us.ihmc.humanoidBehaviors.behaviors;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.producers.CompressedVideoDataClient;
import us.ihmc.communication.producers.CompressedVideoDataFactory;
import us.ihmc.communication.producers.VideoStreamer;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;

public class BlobDetectionBehavior extends BehaviorInterface implements VideoStreamer
{
   private final DoubleYoVariable yoTime;
   private BooleanYoVariable blobFound = new BooleanYoVariable("blobFound", registry);
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<>();
   private final CompressedVideoDataClient videoDataClient;
//   private final HueSaturationValueRange hsvRange; // depends on IHMCPerception

   private BufferedImage latestBufferedImage;
   private Point3d latestCameraPosition;
   private Quat4d latestCameraOrientation;
   private IntrinsicParameters latestCameraParameters;

   public BlobDetectionBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.yoTime = yoTime;

      videoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(this);
      attachNetworkProcessorListeningQueue(videoPacketQueue, VideoPacket.class);
   }

   @Override public void doControl()
   {
      if (videoPacketQueue.isNewPacketAvailable())
      {
         VideoPacket latestVideoPacket = videoPacketQueue.getNewestPacket();
         videoDataClient.consumeObject(latestVideoPacket.getData(), latestVideoPacket.getPosition(), latestVideoPacket.getOrientation(),
               latestVideoPacket.getIntrinsicParameters());

         trackBlob();
      }
   }

   private void trackBlob()
   {
      // TODO detect blob
   }

   @Override protected void passReceivedNetworkProcessorObjectToChildBehaviors(Object object)
   {

   }

   @Override protected void passReceivedControllerObjectToChildBehaviors(Object object)
   {

   }

   @Override
   public void stop()
   {
      defaultStop();
   }

   @Override public void enableActions()
   {

   }

   @Override
   public void pause()
   {
      defaultPause();
   }

   @Override
   public void resume()
   {
      defaultResume();
   }


   @Override public boolean isDone()
   {
      return blobFound.getBooleanValue();
   }

   @Override public void doPostBehaviorCleanup()
   {
      defaultPostBehaviorCleanup();
      blobFound.set(false);
   }

   @Override
   public void initialize()
   {
      defaultPostBehaviorCleanup();
   }

   @Override public boolean hasInputBeenSet()
   {
      return false;
   }

   @Override public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParamaters)
   {
      this.latestBufferedImage = bufferedImage;
      this.latestCameraPosition = cameraPosition;
      this.latestCameraOrientation = cameraOrientation;
      this.latestCameraParameters = intrinsicParamaters;
   }
}
