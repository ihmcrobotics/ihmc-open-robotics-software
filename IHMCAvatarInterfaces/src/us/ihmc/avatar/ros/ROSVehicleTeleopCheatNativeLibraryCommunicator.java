package us.ihmc.avatar.ros;

import java.net.InetAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;

import org.apache.commons.lang3.SystemUtils;

import us.ihmc.avatar.ros.messages.ClockMessage;
import us.ihmc.avatar.ros.messages.Float64Message;
import us.ihmc.avatar.ros.messages.Int8Message;
import us.ihmc.avatar.ros.messages.PoseMessage;
import us.ihmc.utilities.ros.RosTools;

public class ROSVehicleTeleopCheatNativeLibraryCommunicator
{
   private static ROSVehicleTeleopCheatNativeLibraryCommunicator instance = null;
   private static String rosMasterURI;
   
   private static boolean hasNativeLibrary;
   static
   {
      try
      {
         System.loadLibrary("ROSVehicleTeleopCheatNativeLibraryCommunicator");
         hasNativeLibrary = true;
      }
      catch(UnsatisfiedLinkError e)
      {
         System.err.println("Cannot load native ROS library for the vehicle teleop. Falling back to ROSJava, expect poor performance.");
         if (SystemUtils.IS_OS_LINUX)
         {
            System.err.println("Running on a Linux system; Library should be loadable.");
            e.printStackTrace();
         }
         hasNativeLibrary = false;
      }
      
   }

   public static boolean hasNativeLibrary()
   {
      return hasNativeLibrary;
   }

   /*
    * Subscriber Buffers
    */
   private final DoubleBuffer vehiclePoseBuffer;
   private final LongBuffer clockBuffer;

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
    * Listeners
    */

   private final ArrayList<ClockListener> clockListeners = new ArrayList<ClockListener>();
   private final ArrayList<VehiclePoseListener> vehiclePoseListeners = new ArrayList<VehiclePoseListener>();
   private final ArrayList<HandBrakeStateListener> handBrakeStateListeners = new ArrayList<HandBrakeStateListener>();
   private final ArrayList<SteeringWheelStateListener> steeringWheelStateListeners = new ArrayList<SteeringWheelStateListener>();
   private final ArrayList<GasPedalStateListener> gasPedalStateListeners = new ArrayList<GasPedalStateListener>();
   private final ArrayList<BrakePedalStateListener> brakePedalStateListeners = new ArrayList<BrakePedalStateListener>();

   public void addClockListener(ClockListener listener)
   {
      clockListeners.add(listener);
   }

   public void addVehiclePoseListener(VehiclePoseListener listener)
   {
      vehiclePoseListeners.add(listener);
   }

   public void addSteeringWheelStateListener(SteeringWheelStateListener listener)
   {
      steeringWheelStateListeners.add(listener);
   }

   public void addHandBrakeStateListener(HandBrakeStateListener listener)
   {
      handBrakeStateListeners.add(listener);
   }

   public void addGasPedalStateListener(GasPedalStateListener listener)
   {
      gasPedalStateListeners.add(listener);
   }

   public void addBrakePedalStateListener(BrakePedalStateListener listener)
   {
      brakePedalStateListeners.add(listener);
   }

   private ROSVehicleTeleopCheatNativeLibraryCommunicator(String rosMasterURI)
   {
      InetAddress myIP = RosTools.getMyIP(rosMasterURI);

      

      if (!register(rosMasterURI, myIP.getHostAddress()))
      {
         throw new RuntimeException("Cannot load native library");
      }

      vehiclePoseBuffer = setupDoubleBuffer(getVehiclePoseBuffer());
      clockBuffer = setupLongBuffer(getClockBuffer());
      atlasTeleportPoseCommandBuffer = setupDoubleBuffer(getAtlasTeleportPoseBuffer());
      
      Runtime.getRuntime().addShutdownHook(new Thread() {
         @Override
         public void run()
         {
            shutdown();
         }
      });
   }

   public static ROSVehicleTeleopCheatNativeLibraryCommunicator getInstance(String rosMasterURI)
   {
      if (instance == null)
      {
         instance = new ROSVehicleTeleopCheatNativeLibraryCommunicator(rosMasterURI);
      }
      else if (!rosMasterURI.equals(ROSVehicleTeleopCheatNativeLibraryCommunicator.rosMasterURI))
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

   private static LongBuffer setupLongBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer.asLongBuffer();
   }

   private static ByteBuffer setupByteBuffer(ByteBuffer buffer)
   {
      buffer.order(ByteOrder.nativeOrder());

      return buffer;
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

   public void enableOutput()
   {
      enableOutputNative();
   }

   /*
    * Do not remove due to non-use! Invoked by native library!
    */
   @SuppressWarnings("UnusedDeclaration")
   private void receivedClockMessage()
   {
      clock.setFromBuffer(clockBuffer);
      for (int i = 0; i < clockListeners.size(); i++)
      {
         clockListeners.get(i).receivedClockMessage(clock);
      }
   }

   /*
    * Do not remove due to non-use! Invoked by native library!
    */
   @SuppressWarnings("UnusedDeclaration")
   private void receivedVehiclePose()
   {
      vehiclePose.setFromBuffer(vehiclePoseBuffer);
      for (int i = 0; i < vehiclePoseListeners.size(); i++)
      {
         vehiclePoseListeners.get(i).receivedVehiclePose(vehiclePose);
      }
   }

   /*
    * Do not remove due to non-use! Invoked by native library!
    */
   @SuppressWarnings("UnusedDeclaration")
   private void receivedHandBrakeState()
   {
      handBrakeState.setValue(getHandBrakeState());

      for (int i = 0; i < handBrakeStateListeners.size(); i++)
      {
         handBrakeStateListeners.get(i).receivedHandBrakeState(handBrakeState);
      }
   }

   /*
    * Do not remove due to non-use! Invoked by native library!
    */
   @SuppressWarnings("UnusedDeclaration")
   private void receivedSteeringWheelState()
   {
      steeringWheelState.setValue(getSteeringWheelState());
      for (int i = 0; i < steeringWheelStateListeners.size(); i++)
      {
         steeringWheelStateListeners.get(i).receivedSteeringWheelState(steeringWheelState);
      }
   }

   /*
    * Do not remove due to non-use! Invoked by native library!
    */
   @SuppressWarnings("UnusedDeclaration")
   private void receivedGasPedalState()
   {
      gasPedalState.setValue(getGasPedalState());

      for (int i = 0; i < gasPedalStateListeners.size(); i++)
      {
         gasPedalStateListeners.get(i).receivedGasPedalState(gasPedalState);
      }
   }

   /*
    * Do not remove due to non-use! Invoked by native library!
    */
   @SuppressWarnings("UnusedDeclaration")
   private void receivedBrakePedalState()
   {
      brakePedalState.setValue(getBrakePedalState());

      for (int i = 0; i < brakePedalStateListeners.size(); i++)
      {
         brakePedalStateListeners.get(i).receivedBrakePedalState(brakePedalState);
      }
   }

   private native boolean register(String rosMasterURI, String myIP);

   private native void spin();

   public native double getSteeringWheelState();

   public native double getHandBrakeState();

   public native double getGasPedalState();

   public native double getBrakePedalState();

   private native ByteBuffer getVehiclePoseBuffer();

   private native ByteBuffer getAtlasTeleportPoseBuffer();

   private native ByteBuffer getClockBuffer();

   private native void sendDirectionCommand(byte directionCommand, long timestamp, int delay);

   private native void sendSteeringWheelCommand(double steeringWheelCommand, long timestamp, int delay);

   private native void sendHandBrakeCommand(double handBrakeCommand, long timestamp, int delay);

   private native void sendGasPedalCommand(double gasPedalCommand, long timestamp, int delay);

   private native void sendBrakePedalCommand(double brakePedalCommand, long timestamp, int delay);

   private native void sendAtlasTeleportIntoVehicleCommand(long timestamp, int delay);

   private native void sendAtlasTeleportOutOfVehicleCommand(long timestamp, int delay);

   private native void enableOutputNative();
   
   private native void shutdown();
}
