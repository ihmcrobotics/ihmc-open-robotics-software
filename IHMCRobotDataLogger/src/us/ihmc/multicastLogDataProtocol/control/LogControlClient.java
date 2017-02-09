package us.ihmc.multicastLogDataProtocol.control;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class LogControlClient implements NetStateListener
{
   private final KryoObjectClient kryoObjectClient;

   private YoVariablesUpdatedListener yoVariablesUpdatedListener = null;
   private final List<LogControlVariableChangeListener> logControlVariableChangeListeners = new ArrayList<>();

   public LogControlClient(byte[] host, int port, int writeBufferSize, int readBufferSize)
   {
      try
      {
         InetAddress address = InetAddress.getByAddress(host);
         NetClassList list = new LogControlClassList();
         kryoObjectClient = new KryoObjectClient(address, port, list, writeBufferSize, readBufferSize);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }

   public LogControlClient(byte[] host, int port, YoVariablesUpdatedListener listener)
   {
      this(host, port, 2097152, 2097152);
      this.yoVariablesUpdatedListener = listener;

      kryoObjectClient.attachStateListener(this);
      kryoObjectClient.attachListener(ClearLogRequest.class, new ClearLogRequestListener());
   }

   @Override
   public void connected()
   {
   }

   @Override
   public void disconnected()
   {
   }

   public void close()
   {
      if (kryoObjectClient.isConnected())
      {
         kryoObjectClient.close();
      }
   }

   public void connect()
   {
      try
      {
         kryoObjectClient.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }
   
   public void setSendingChangesEnabled(boolean sendingChangesEnabled)
   {
      for (int i = 0; i < logControlVariableChangeListeners.size(); i++)
      {
         logControlVariableChangeListeners.get(i).setSendingChangesEnabled(sendingChangesEnabled);
      }
   }

   public void startVariableChangedProducers(List<YoVariable<?>> variables, boolean sendChangedValues)
   {
      for (int i = 0; i < variables.size(); i++)
      {
         LogControlVariableChangeListener logControlVariableChangeListener = new LogControlVariableChangeListener(i, sendChangedValues);
         variables.get(i).addVariableChangedListener(logControlVariableChangeListener);
         logControlVariableChangeListeners.add(logControlVariableChangeListener);
      }
   }

   public void sendClearLogRequest()
   {
      kryoObjectClient.consumeObject(new ClearLogRequest());
   }

   public class LogControlVariableChangeListener implements VariableChangedListener
   {
      private final int id;
      private boolean sendChangedValue;

      public LogControlVariableChangeListener(int id, boolean sendChangedValue)
      {
         this.id = id;
         this.sendChangedValue = sendChangedValue;
      }

      @Override
      public void variableChanged(YoVariable<?> v)
      {
         if (sendChangedValue)
         {
            VariableChangeRequest request = new VariableChangeRequest();
            request.variableID = id;
            request.requestedValue = v.getValueAsDouble();
            kryoObjectClient.consumeObject(request);
         }
      }
      
      public void setSendingChangesEnabled(boolean sendingChangesEnabled)
      {
         sendChangedValue = sendingChangesEnabled;
      }
   }

   public class ClearLogRequestListener implements ObjectConsumer<ClearLogRequest>
   {
      @Override
      public void consumeObject(ClearLogRequest object)
      {
         yoVariablesUpdatedListener.clearLog();
      }
   }
}
