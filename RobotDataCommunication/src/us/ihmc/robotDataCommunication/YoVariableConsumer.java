package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.CRC32;

import us.ihmc.multicastLogDataProtocol.LogPacketHandler;
import us.ihmc.multicastLogDataProtocol.StreamingDataTCPClient;
import us.ihmc.multicastLogDataProtocol.ThreadedLogPacketHandler;
import us.ihmc.multicastLogDataProtocol.control.LogControlClient.LogControlVariableChangeListener;
import us.ihmc.multicastLogDataProtocol.control.LogHandshake;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.tools.compression.SnappyUtils;

public class YoVariableConsumer extends YoVariableDataReceiver
{
   private final List<YoVariable<?>> variables;
   private final List<JointState<?>> jointStates;
   private final YoVariablesUpdatedListener listener;

   public YoVariableConsumer(byte[] dataIP, int port, List<YoVariable<?>> variables, List<JointState<?>> jointStates, YoVariablesUpdatedListener listener)
   {
      super(dataIP, port, listener.getDisplayOneInNPackets());

      this.variables = variables;
      this.jointStates = jointStates;
      this.listener = listener;
   }

   @Override
   public void onNewData(ByteBuffer decompressed)
   {
      long timestamp = decompressed.getLong();
      LongBuffer data = decompressed.asLongBuffer();

      for (int i = 0; i < variables.size(); i++)
      {
         YoVariable<?> variable = variables.get(i);
         long previousValue = variable.getValueAsLongBits();
         long newValue = data.get();
         variable.setValueFromLongBits(newValue, false);
         if (previousValue != newValue)
         {
            ArrayList<VariableChangedListener> changedListeners = variable.getVariableChangedListeners();
            if (changedListeners != null)
            {
               for (int listener = 0; listener < changedListeners.size(); listener++)
               {
                  VariableChangedListener changedListener = changedListeners.get(listener);
                  if (!(changedListener instanceof LogControlVariableChangeListener))
                  {
                     changedListener.variableChanged(variable);
                  }
               }
            }
         }
      }

      for (int i = 0; i < jointStates.size(); i++)
      {
         jointStates.get(i).update(data);
      }

      listener.receivedTimestampAndData(timestamp, decompressed);
   }

   @Override
   public void onNewTimestamp(long timestamp)
   {
      listener.receivedTimestampOnly(timestamp);
   }

   @Override
   public void onTimeout()
   {
      listener.receiveTimedOut();
   }
}