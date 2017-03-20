package us.ihmc.wholeBodyController;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.concurrent.locks.ReentrantLock;

import javax.imageio.ImageIO;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.CalibrateArmPacket;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.sensorProcessing.sensorData.CameraData;
import us.ihmc.sensorProcessing.sensorData.DRCStereoListener;

public class ArmCalibrationHelper implements DRCStereoListener, PacketConsumer<CalibrateArmPacket>
{
   private final static String basedir = "/tmp/calibration";

   private final ReentrantLock lock = new ReentrantLock();

   private BufferedImage lastLeftEyeImage;
   private long imageTimestamp;
   private final DRCRobotJointMap jointMap;

   private RobotConfigurationData lastJointConfigurationData;

   public ArmCalibrationHelper(PacketCommunicator fieldComputerClient, PacketCommunicator networkingManager,
         DRCRobotJointMap jointMap)
   {
      this.jointMap = jointMap;
      networkingManager.attachListener(CalibrateArmPacket.class, this);
      fieldComputerClient.attachListener(RobotConfigurationData.class, new JointAngleConsumer());
      startPeriodicRecording();
   }
   
   public void startPeriodicRecording()
   {
      new Thread(new Runnable()
      {
         
         @Override
         public void run()
         {
            // TODO Auto-generated method stub
            while(true)
            {
               try
               {
                  Thread.sleep(1000);
               }
               catch (InterruptedException e)
               {
                  // TODO Auto-generated catch block
                  e.printStackTrace();
               }
               calibrateArm();
            }
         }
      }).start();;
   }

   @Override
   public void newImageAvailable(CameraData data, RigidBodyTransform transformToCamera)
   {
      if(data.videoSource == VideoSource.MULTISENSE_LEFT_EYE)
      {
         lock.lock();
         lastLeftEyeImage = data.image;
         imageTimestamp = data.timestamp;
         lock.unlock();
      }
   }


   private class JointAngleConsumer implements PacketConsumer<RobotConfigurationData>
   {
      @Override
      public void receivedPacket(RobotConfigurationData object)
      {
         lock.lock();
         lastJointConfigurationData = object;
         lock.unlock();
      }
   }

   public void calibrateArm()
   {
      lock.lock();
      try
      {
         if (lastLeftEyeImage == null || lastJointConfigurationData == null)
         {
            System.err.println("Did not receive image and robotdata yet");
            return;
         }
         

         File captureDir = new File(basedir, String.valueOf(lastJointConfigurationData.getTimestamp()));
         captureDir.mkdirs();
         System.out.println("Writing calibration files to " + captureDir);
         File image = new File(captureDir, "leftEyeImage.png");
         File imageData = new File(captureDir, "imageData.m");
         File q = new File(captureDir, "q.m");
         File qout = new File(captureDir, "qout.m");

         ImageIO.write(lastLeftEyeImage, "png", image);
         PrintWriter imageDataWriter = new PrintWriter(imageData);
         imageDataWriter.print("timestamp = ");
         imageDataWriter.println(imageTimestamp);
         imageDataWriter.print("fov = ");
         imageDataWriter.println("FIXME");
         imageDataWriter.close();

         PrintWriter qWriter = new PrintWriter(q);
//         PrintWriter qoutWriter = new PrintWriter(qout);

         
         System.err.println("TODO: Reimplement ArmCalibrationManager to write joint names from SDF");
//         String[] jointNames = lastJointConfigurationData.getJointNames();
//         for (int i = 0; i < jointNames.length; i++)
//         {
//            qWriter.print(jointNames[i]);
//            qWriter.print("=");
//            qWriter.println(lastJointConfigurationData.getJointAngles()[i]);
//
////            qoutWriter.print(jointNames[i]);
////            qoutWriter.print("=");
////            qoutWriter.println(lastJointConfigurationData.getJointOutAngles()[i]);
//         }

         qWriter.close();
//         qoutWriter.close();
         System.out.println("Wrote calibration files");
      }
      catch (IOException e)
      {
         System.err.println("Cannot write joint angles");
         e.printStackTrace();
      }
      finally
      {
         lock.unlock();
      }
   }

   public void receivedPacket(CalibrateArmPacket object)
   {
      calibrateArm();
   }

}
