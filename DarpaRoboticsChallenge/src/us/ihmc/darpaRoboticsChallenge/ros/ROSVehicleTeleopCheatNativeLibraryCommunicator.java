package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.networkProcessor.ros.RosTools;

import java.net.InetAddress;
import java.nio.*;
import java.util.concurrent.locks.ReentrantLock;

public class ROSVehicleTeleopCheatNativeLibraryCommunicator
{
   private static ROSVehicleTeleopCheatNativeLibraryCommunicator instance = null;
   private static String rosMasterURI;

   /*
    * Subscribers
    */
   private final DoubleBuffer vehiclePoseBuffer;
   private final ByteBuffer leftEyeImageBuffer, rightEyeImageBuffer;
   private final LongBuffer clockBuffer;
   private double steeringWheelState, handBrakeState, gasPedalState, brakePedalState;

   /*
    * Publishers
    */
   private byte directionCommandBuffer;
   private double steeringWheelCommandBuffer, handBrakeCommandBuffer, gasPedalCommandBuffer, brakePedalCommandBuffer;
   private final DoubleBuffer atlasTeleportPoseCommandBuffer;

   /*
    * TODO: Make classes for messages
    *
    * PoseMessage (vehiclePose, atlasPose)
    * ImageMessage
    * ClockMessage
    * Float64Message
    * Int8Message
    *
    */

   private final ReentrantLock lock =  new ReentrantLock();

   private ROSVehicleTeleopCheatNativeLibraryCommunicator(String rosMasterURI)
   {
      InetAddress myIP = RosTools.getMyIP(rosMasterURI);

      System.loadLibrary("ROSVehicleTeleopCheatNativeLibraryCommunicator");
      if (!register(rosMasterURI, myIP.getHostAddress()))
      {
         throw new RuntimeException("Cannot load native library");
      }
      vehiclePoseBuffer = null;
      leftEyeImageBuffer = null;
      rightEyeImageBuffer = null;
      clockBuffer = null;
      atlasTeleportPoseCommandBuffer = null;
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

   private static DoubleBuffer setupDoubleBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer.asDoubleBuffer();
   }

   private static ByteBuffer setUpByteBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer;
   }

   private static LongBuffer setUpLongBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer.asLongBuffer();
   }

   private static FloatBuffer setupFloatBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return  buffer.asFloatBuffer();
   }

   private native boolean register(String rosMasterURI, String myIP);

   private native void spin();

}
