package us.ihmc.avatar.stereoVision.sensor;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.spinnaker.Spinnaker_C.spinImage;
import us.ihmc.avatar.stereoVision.ImageCropInfo;
import us.ihmc.log.LogTools;
import us.ihmc.perception.spinnaker.SpinnakerBlackfly;
import us.ihmc.perception.spinnaker.SpinnakerBlackflyManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.concurrent.atomic.AtomicInteger;

// TODO: should we be synchronizing the cameras?
public class StereoVisionBlackflyReader extends StereoVisionSensorReader
{
   private SpinnakerBlackflyManager spinnakerBlackflyManager;
   private final SideDependentList<AtomicInteger> frameNumbers = new SideDependentList<>();
   private final SideDependentList<BlackflyReadThread> blackflyReadThreads = new SideDependentList<>();

   public StereoVisionBlackflyReader()
   {
      frameNumbers.put(RobotSide.LEFT, new AtomicInteger());
      frameNumbers.put(RobotSide.RIGHT, new AtomicInteger());
   }

   @Override
   protected void startInternal(SideDependentList<String> serialNumbers, ImageCropInfo imageCropInfo)
   {
      spinnakerBlackflyManager = new SpinnakerBlackflyManager();

      for (RobotSide side : RobotSide.values)
      {
         // Reset frame number
         frameNumbers.get(side).set(0);

         BlackflyReadThread blackflyReadThread = new BlackflyReadThread(side, serialNumbers.get(side), imageCropInfo);
         blackflyReadThreads.put(side, blackflyReadThread);
         blackflyReadThread.start();
      }
   }

   @Override
   protected void stopInternal()
   {
      for (BlackflyReadThread blackflyReadThread : blackflyReadThreads)
      {
         if (blackflyReadThread != null)
         {
            try
            {
               blackflyReadThread.join();
            }
            catch (InterruptedException e)
            {
               LogTools.error(e);
            }
         }
      }

      spinnakerBlackflyManager.destroy();
   }

   private class BlackflyReadThread extends Thread
   {
      private final RobotSide side;
      private final String serialNumber;
      private final ImageCropInfo imageCropInfo;

      public BlackflyReadThread(RobotSide side, String serialNumber, ImageCropInfo imageCropInfo)
      {
         this.side = side;
         this.serialNumber = serialNumber;
         this.imageCropInfo = imageCropInfo;
      }

      @Override
      public void run()
      {
         SpinnakerBlackfly spinnakerBlackfly = spinnakerBlackflyManager.createSpinnakerBlackfly(serialNumber,
                                                                                                imageCropInfo.getCropWidth(),
                                                                                                imageCropInfo.getCropHeight(),
                                                                                                imageCropInfo.getXOffset(),
                                                                                                imageCropInfo.getYOffset());
         while (isRunning())
         {
            spinImage spinImage = new spinImage();

            spinnakerBlackfly.getNextImage(spinImage);

            int width = spinnakerBlackfly.getWidth(spinImage);
            int height = spinnakerBlackfly.getHeight(spinImage);
            int imageFrameSize = width * height;

            BytePointer spinImageData = new BytePointer(imageFrameSize);
            spinnakerBlackfly.setPointerToSpinImageData(spinImage, spinImageData);

            int imageDataLength = (int) spinImageData.limit();
            byte[] fullImageData = new byte[imageDataLength];
            spinImageData.get(fullImageData, 0, imageDataLength);

            int frameNumber = frameNumbers.get(side).getAndIncrement();

            for (StereoVisionSensorReaderListener listener : listeners)
               listener.onNewImage(side, frameNumber, width, height, fullImageData);

            spinnakerBlackfly.releaseImage(spinImage);
         }

         spinnakerBlackfly.stopAcquiringImages();
      }
   }
}
