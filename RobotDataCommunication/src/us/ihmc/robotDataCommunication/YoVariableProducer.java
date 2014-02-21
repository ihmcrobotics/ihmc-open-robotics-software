package us.ihmc.robotDataCommunication;

import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.List;

import org.zeromq.ZMQ;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotDataCommunication.jointState.JointHolder;

import com.yobotics.simulationconstructionset.YoVariable;

public class YoVariableProducer extends Thread
{
   private final zmq.Ctx ctx = zmq.ZMQ.zmq_init(1);
   
   private final int port;
   private final ConcurrentRingBuffer<FullStateBuffer> variableBuffer;
   private final byte[] buffer;
   private final byte[][] outputBuffer;
   private final int outputBufferSize=8; // 0 disables outputBuffer
   private final int jointStateOffset;
   private final LongBuffer longBuffer;
   private zmq.Msg msg;

   private zmq.SocketBase variablePublisher;
   
   public YoVariableProducer(int port, List<YoVariable> variables, List<JointHolder> jointHolders, ConcurrentRingBuffer<FullStateBuffer> variableBuffer)
   {
      super("YoVariableProducer_" + port);
      this.port = port;
      this.variableBuffer = variableBuffer;
      
      this.jointStateOffset = variables.size();
      int numberOfJointStates = FullStateBuffer.getNumberOfJointStates(jointHolders);
      int bufferSize = (1 + jointStateOffset + numberOfJointStates) * 8;
      this.buffer = new byte[bufferSize];
      
      this.outputBuffer=new byte[outputBufferSize][];
      for(int i=0;i<outputBufferSize;i++)
      {
         this.outputBuffer[i] = new byte[bufferSize];         
      }
      this.msg = new zmq.Msg(buffer);
      
      ByteBuffer byteBuffer = ByteBuffer.wrap(buffer);
      this.longBuffer = byteBuffer.asLongBuffer();
      
      longBuffer.put(0);
      for(int i = 0; i < variables.size(); i++)
      {
         longBuffer.put(variables.get(i).getValueAsLongBits());
      }
   }
   
   public void run()
   {
      variablePublisher = ctx.create_socket(ZMQ.PUB);
      RemoteVisualizationUtils.mayRaise();

      if (!variablePublisher.bind("tcp://*:" + port))
      {
         RemoteVisualizationUtils.mayRaise();
      }
      
      
      while (true)
      {
         variableBuffer.poll();
         FullStateBuffer fullStateBuffer;
         int outputBufferIndex=0;
         while((fullStateBuffer = variableBuffer.read()) != null)
         {
            longBuffer.put(0, fullStateBuffer.getTimestamp());
            for(int i = 0; i < jointStateOffset; i++)
            {
               if(fullStateBuffer.hasChangedAndReset(i))
               {
                  longBuffer.put(i + 1, fullStateBuffer.getValue(i));                  
               }
            }
            
            fullStateBuffer.getJointStatesInBuffer(longBuffer, jointStateOffset + 1);
            
            if(outputBufferSize>0)
            {
               System.arraycopy(buffer, 0, outputBuffer[outputBufferIndex], 0, buffer.length);
               msg = new zmq.Msg(outputBuffer[outputBufferIndex]);
               outputBufferIndex = (outputBufferIndex+1)%outputBufferSize;
            }
            
            variablePublisher.send(msg, 0);
         }
         variableBuffer.flush();
         
         try
         {
            Thread.sleep(1);
         }
         catch (InterruptedException e)
         {
            break;
         }
      }
   
      if (variablePublisher.check_tag()) variablePublisher.close(); // don't close if already closed
   }
   
   public void close()
   {
      if (variablePublisher != null)
      {
         variablePublisher.close();
      }
      ctx.terminate();
      interrupt();
      try
      {
         join();
      }
      catch (InterruptedException e)
      {
      }
   }
}