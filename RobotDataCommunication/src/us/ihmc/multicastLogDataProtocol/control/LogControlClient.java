package us.ihmc.multicastLogDataProtocol.control;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.List;

import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.robotDataCommunication.YoVariablesUpdatedListener;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class LogControlClient implements NetStateListener
{
   private final KryoObjectClient client;
   private final YoVariablesUpdatedListener listener;

   public LogControlClient(byte[] host, int port, YoVariablesUpdatedListener listener)
   {
      this.listener = listener;
      try
      {
         client = new KryoObjectClient(InetAddress.getByAddress(host), port, new LogControlClassList(), 2097152, 2097152);
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException(e);
      }
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
      // TODO Auto-generated method stub

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

   public void startVariableChangedProducers(List<YoVariable<?>> variables)
   {
      for (int i = 0; i < variables.size(); i++)
      {
         LogControlVariableChangeListener variableChangedListener = new LogControlVariableChangeListener(i);
         variables.get(i).addVariableChangedListener(variableChangedListener);
      }
   }

   public class LogControlVariableChangeListener implements VariableChangedListener
   {
      private final int id;

      public LogControlVariableChangeListener(int id)
      {
         this.id = id;
      }

      public void variableChanged(YoVariable<?> v)
      {
         VariableChangeRequest request = new VariableChangeRequest();
         request.variableID = id;
         request.requestedValue = v.getValueAsDouble();
         client.consumeObject(request);
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

   public void sendClearLogRequest()
   {
      client.consumeObject(new ClearLogRequest());
   }
}
