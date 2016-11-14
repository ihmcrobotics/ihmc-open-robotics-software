package us.ihmc.humanoidBehaviors.behaviors.qrCode;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class LocateQRCodeBehavior extends AbstractBehavior
{
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>();

   private final BooleanYoVariable foundCode = new BooleanYoVariable("FoundCode", registry);
   private final FramePose codePose = new FramePose(ReferenceFrame.getWorldFrame());

   public LocateQRCodeBehavior(CommunicationBridgeInterface communicationBridge)
   {
      super(communicationBridge);
      attachNetworkListeningQueue(videoPacketQueue, VideoPacket.class);
   }

   @Override
   public void doControl()
   {
      if (!videoPacketQueue.isNewPacketAvailable())
         return;

      VideoPacket videoPacket = videoPacketQueue.getLatestPacket();
      // locate code...

      codePose.setPose(new Point3d(2.0, 0.0, 0.0), new Quat4d(0.0, 0.0, 0.0, 1.0));
      foundCode.set(true);
   }

   @Override
   public boolean isDone()
   {
      return foundCode.getBooleanValue();
   }

   @Override
   public void initialize()
   {
      super.initialize();
      foundCode.set(false);
      codePose.setToNaN();
   }

   public FramePose getCodePose()
   {
      return codePose;
   }

}
