package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.nio.channels.Selector;
import java.util.ArrayList;
import java.util.List;

import org.jvcompress.util.LZOUtils;
import org.jvcompress.util.MInt;
import org.zeromq.ZMQ;

import us.ihmc.robotDataCommunication.YoVariableChangedProducer.YoVariableClientChangedListener;
import us.ihmc.robotDataCommunication.jointState.JointState;
import us.ihmc.robotDataCommunication.jointState.OneDoFState;
import zmq.PollItem;
import zmq.ZError;

import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;

public class YoVariableConsumer extends Thread
{

   private final zmq.Ctx ctx = zmq.ZMQ.zmq_init(1);

   private final String host;
   private final int port;
   
   private final List<YoVariable> variables;
   private final List<JointState<?>> jointStates;
   private final YoVariablesUpdatedListener listener;
   
   private volatile boolean connected = false;
   
   private long packetNumber = 0;

   public YoVariableConsumer(String host, int port, List<YoVariable> variables, List<JointState<?>> jointStates, YoVariablesUpdatedListener listener)
   {
      super();
      this.host = host;
      this.port = port;
      
      this.variables = variables;
      this.jointStates = jointStates;
      this.listener = listener;
    
   }
   
   @Override
   public void run()
   {
      zmq.SocketBase socketBase = ctx.create_socket(ZMQ.SUB);
      RemoteVisualizationUtils.mayRaise(socketBase);

      if (!socketBase.connect("tcp://" + host + ":" + port))
      {
         RemoteVisualizationUtils.mayRaise(socketBase);
      }

      socketBase.setsockopt(zmq.ZMQ.ZMQ_SUBSCRIBE, new byte[0]);
      
      RemoteVisualizationUtils.mayRaiseNot(socketBase, ZError.ETERM);
      PollItem[] pollItem = { new PollItem(socketBase, zmq.ZMQ.ZMQ_POLLIN) };
      Selector selector;
      try
      {
         selector = Selector.open();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      int numberOfJointStates = OneDoFState.getNumberOfJointStates(jointStates);
      int bufferSize = (1 + variables.size() + numberOfJointStates) * 8;
      ByteBuffer decompressed = ByteBuffer.allocate(bufferSize);
      
      connected = true;
      
      MInt outputLength = LZOUtils.createOutputLength();
      while (connected)
      {
         
         boolean readable = false;
         do
         {
            int receiveTimeout = 10;
            zmq.ZMQ.zmq_poll(selector, pollItem, receiveTimeout);
            readable = pollItem[0].isReadable();
            if(!readable)
            {
               listener.receiveTimedOut(receiveTimeout);
            }
         }
         while(connected && !readable);
         
         
         zmq.Msg msg = socketBase.recv(zmq.ZMQ.ZMQ_NOBLOCK);
         
         if(msg != null && packetNumber++ % listener.getDisplayOneInNPackets() == 0)
         {
            ByteBuffer buf = msg.buf();
            decompressed.clear();
            buf.clear();
            try
            {
               LZOUtils.decompress(buf, decompressed, outputLength);
            }
            catch (Exception e)
            {
               e.printStackTrace();
               continue;
            }
            
            long timestamp = decompressed.getLong();
            LongBuffer data = decompressed.asLongBuffer();

            for (int i = 0; i < variables.size(); i++)
            {
               YoVariable variable = variables.get(i);
               long previousValue = variable.getValueAsLongBits();
               long newValue = data.get();
               variable.setValueFromLongBits(newValue, false);
               if (previousValue != newValue)
               {
                  ArrayList<VariableChangedListener> changedListeners = variable.getVariableChangedListeners();
                  if(changedListeners != null)
                  {
                     for (int listener = 0; listener < changedListeners.size(); listener++)
                     {
                        VariableChangedListener changedListener = changedListeners.get(listener);
                        if (!(changedListener instanceof YoVariableClientChangedListener))
                        {
                           changedListener.variableChanged(variable);
                        }
                     }
                  }
               }
            }
            
            for(int i = 0; i < jointStates.size(); i++)
            {
               jointStates.get(i).update(data);
            }
            
            
            listener.receivedUpdate(timestamp, buf);
         }
      }
      socketBase.close();
      ctx.terminate();
   }
   
   public void close()
   {
      connected = false;
      
      if(Thread.currentThread() != this)
      {
         try
         {
            join();
         }
         catch (InterruptedException e)
         {
         }
      }
      
   }
}