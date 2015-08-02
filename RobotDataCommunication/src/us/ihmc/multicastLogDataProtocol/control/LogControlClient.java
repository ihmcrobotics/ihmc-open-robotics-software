package us.ihmc.multicastLogDataProtocol.control;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.List;

import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.robotDataCommunication.YoVariablesUpdatedListener;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class LogControlClient implements NetStateListener
{
   private final KryoObjectClient client;

   private YoVariablesUpdatedListener listener = null;

   public LogControlClient(byte[] host, int port, int writeBufferSize, int readBufferSize)
   {
      try
      {
         InetAddress address = InetAddress.getByAddress(host);
         NetClassList list = new LogControlClassList();
         client = new KryoObjectClient(address, port, list, writeBufferSize, readBufferSize);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
   }

   public LogControlClient(byte[] host, int port, YoVariablesUpdatedListener listener)
   {
      this(host, port, 2097152, 2097152);
      this.listener = listener;

      client.attachStateListener(this);
      client.attachListener(ClearLogRequest.class, new ClearLogRequestListener());
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
      if (client.isConnected())
      {
         client.close();
      }
   }

   public void connect()
   {
      try
      {
         client.connect();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public void startVariableChangedProducers(List<YoVariable<?>> variables, boolean sendChangedValues)
   {
      for (int i = 0; i < variables.size(); i++)
      {
         LogControlVariableChangeListener variableChangedListener = new LogControlVariableChangeListener(i, sendChangedValues);
         variables.get(i).addVariableChangedListener(variableChangedListener);
      }
   }

   public void sendClearLogRequest()
   {
      client.consumeObject(new ClearLogRequest());
   }

   public void sendVariableChangeRequest(int id, double value)
   {
      VariableChangeRequest request = new VariableChangeRequest();
      request.variableID = id;
      request.requestedValue = value;
      client.consumeObject(request);
   }

   public class LogControlVariableChangeListener implements VariableChangedListener
   {
      private final int id;
      private final boolean sendChangedValue;

      public LogControlVariableChangeListener(int id, boolean sendChangedValue)
      {
         this.id = id;
         this.sendChangedValue = sendChangedValue;
      }

      public void variableChanged(YoVariable<?> v)
      {
         if (sendChangedValue)
         {
            sendVariableChangeRequest(id, v.getValueAsDouble());
         }
      }
   }

   public class ClearLogRequestListener implements ObjectConsumer<ClearLogRequest>
   {
      @Override
      public void consumeObject(ClearLogRequest object)
      {
         listener.clearLog();
      }
   }
}
