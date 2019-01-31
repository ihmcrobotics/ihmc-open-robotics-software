package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.messager.Messager;

public class REAStereoVisionBuffer
{
   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);

   private final REAModuleStateReporter moduleStateReporter;

   private final Messager reaMessager;

   public REAStereoVisionBuffer(Messager reaMessager, REAModuleStateReporter moduleStateReporter)
   {
      this.reaMessager = reaMessager;
      this.moduleStateReporter = moduleStateReporter;

   }

   // TODO : thread for stereo vision buffer.
   public Runnable createBufferThread()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            // TODO : moduleStateReporter will report a 'result' to visualize.
            // TODO : this class will have a method polling the result to REAStereoVisionUpdater.
         }
      };
   }
   
   private void updateStereoVision()
   {
      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = latestStereoVisionPointCloudMessage.getAndSet(null);
      
      if (stereoVisionPointCloudMessage == null)
         return;
      
      // TODO : Build an object which will be used in a method `createBUfferThread`.
   }

   public void handleStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
   }
}
