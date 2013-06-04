package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosTools;
import us.ihmc.darpaRoboticsChallenge.ros.messages.ClockMessage;
import us.ihmc.darpaRoboticsChallenge.ros.messages.Float64Message;
import us.ihmc.darpaRoboticsChallenge.ros.messages.Int8Message;
import us.ihmc.darpaRoboticsChallenge.ros.messages.PoseMessage;

import java.net.InetAddress;
import java.nio.*;
import java.util.concurrent.locks.ReentrantLock;

public class ROSVehicleTeleopCheatNativeLibraryCommunicator
{
   private static ROSVehicleTeleopCheatNativeLibraryCommunicator instance = null;
   private static String rosMasterURI;

   /*
    * Subscriber Buffers
    */
   private final DoubleBuffer vehiclePoseBuffer;

// private final ByteBuffer leftEyeImageBuffer, rightEyeImageBuffer;
   private final IntBuffer clockBuffer;

   /*
    * Subscriber Messages
    */
   private final PoseMessage vehiclePose = new PoseMessage();
   private final ClockMessage clock = new ClockMessage();
   private final Float64Message
      steeringWheelState = new Float64Message(), handBrakeState = new Float64Message(), gasPedalState = new Float64Message(),
      brakePedalState = new Float64Message();

   /*
    * Publisher Buffers
    */
   private final DoubleBuffer atlasTeleportPoseCommandBuffer;

   /*
    * Publisher Messages
    */
   private final PoseMessage atlasTeleportPoseCommand = new PoseMessage();
   private final Int8Message directionCommand = new Int8Message();
   private final Float64Message
      steeringWheelCommand = new Float64Message(), handBrakeCommand = new Float64Message(), gasPedalCommand = new Float64Message(),
      brakePedalCommand = new Float64Message();

   /*
    * TODO: Make classes for messages
    *
    * ImageMessage
    *
    */

   private final ReentrantLock lock = new ReentrantLock();

   private ROSVehicleTeleopCheatNativeLibraryCommunicator(String rosMasterURI)
   {
      InetAddress myIP = RosTools.getMyIP(rosMasterURI);

      System.loadLibrary("ROSVehicleTeleopCheatNativeLibraryCommunicator");

      if (!register(rosMasterURI, myIP.getHostAddress()))
      {
         throw new RuntimeException("Cannot load native library");
      }

      vehiclePoseBuffer = setupDoubleBuffer(getVehiclePoseBuffer());
      clockBuffer = setupIntBuffer(getClockBuffer());
      atlasTeleportPoseCommandBuffer = setupDoubleBuffer(getAtlasTeleportPoseBuffer());
   }

   public static ROSVehicleTeleopCheatNativeLibraryCommunicator getInstance(String rosMasterURI)
   {
      if (instance == null)
      {
         instance = new ROSVehicleTeleopCheatNativeLibraryCommunicator(rosMasterURI);
      }
      else if (!rosMasterURI.equals(""))
      {
         throw new RuntimeException("Cannot get an instance of ROSVehicleTeleopCheatNativeLibraryCommunicator for " + rosMasterURI + ", already connected to "
                                    + ROSVehicleTeleopCheatNativeLibraryCommunicator.rosMasterURI);
      }

      return instance;
   }

   public void connect()
   {
      new Thread(new Runnable()
      {
         public void run()
         {
            spin();
         }
      }).start();
   }

   private static DoubleBuffer setupDoubleBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer.asDoubleBuffer();
   }

   private static IntBuffer setupIntBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer.asIntBuffer();
   }

   public void sendDirectionCommand(long timestamp, int delay, Int8Message directionCommand)
   {
      sendDirectionCommand((byte) directionCommand.getValue(), timestamp, delay);
   }

   public void sendSteeringWheelCommand(long timestamp, int delay, Float64Message steeringWheelCommand)
   {
      sendSteeringWheelCommand(steeringWheelCommand.getValue(), timestamp, delay);
   }

   public void sendHandBrakeCommand(long timestamp, int delay, Float64Message handBrakeCommand)
   {
      sendHandBrakeCommand(handBrakeCommand.getValue(), timestamp, delay);
   }

   public void sendGasPedalCommand(long timestamp, int delay, Float64Message gasPedalCommand)
   {
      sendGasPedalCommand(gasPedalCommand.getValue(), timestamp, delay);
   }

   public void sendBrakePedalCommand(long timestamp, int delay, Float64Message brakePedalCommand)
   {
      sendBrakePedalCommand(brakePedalCommand.getValue(), timestamp, delay);
   }

   public void sendAtlasTeleportIntoVehicleCommand(long timestamp, int delay, PoseMessage atlasTeleportPoseCommand)
   {
      atlasTeleportPoseCommand.copyToBuffer(atlasTeleportPoseCommandBuffer);
      sendAtlasTeleportIntoVehicleCommand(timestamp, delay);
   }

   public void sendAtlasTeleportOutOfVehicleCommand(long timestamp, int delay, PoseMessage atlasTeleportPoseCommand)
   {
      atlasTeleportPoseCommand.copyToBuffer(atlasTeleportPoseCommandBuffer);
      sendAtlasTeleportOutOfVehicleCommand(timestamp, delay);
   }

   private native boolean register(String rosMasterURI, String myIP);

   private native void spin();

   private native ByteBuffer getVehiclePoseBuffer();

   private native ByteBuffer getAtlasTeleportPoseBuffer();

   private native ByteBuffer getClockBuffer();

   private native double getSteeringWheelState();

   private native double getHandBrakeState();

   private native double getGasPedalState();

   private native double getBrakePedalState();

   private native void sendDirectionCommand(byte directionCommand, long timestamp, int delay);

   private native void sendSteeringWheelCommand(double steeringWheelCommand, long timestamp, int delay);

   private native void sendHandBrakeCommand(double handBrakeCommand, long timestamp, int delay);

   private native void sendGasPedalCommand(double gasPedalCommand, long timestamp, int delay);

   private native void sendBrakePedalCommand(double brakePedalCommand, long timestamp, int delay);

   private native void sendAtlasTeleportIntoVehicleCommand(long timestamp, int delay);

   private native void sendAtlasTeleportOutOfVehicleCommand(long timestamp, int delay);
}
