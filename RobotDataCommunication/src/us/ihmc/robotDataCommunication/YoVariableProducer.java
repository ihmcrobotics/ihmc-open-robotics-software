package us.ihmc.robotDataCommunication;

import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.Collection;

import org.zeromq.ZMQ;

import us.ihmc.concurrent.ConcurrentRingBuffer;

public class YoVariableProducer extends Thread
{
   private static final int NUMBER_OF_WRITE_BUFFERS = 12;
   
   private final zmq.Ctx ctx = zmq.ZMQ.zmq_init(1);
   
   private final int port;
   private final ConcurrentRingBuffer<FullStateBuffer> mainBuffer;
   private final ConcurrentRingBuffer<RegistryBuffer>[] buffers;
  
   private final byte[] backingArray;
   private final LongBuffer writeBuffer;
   
   private final int jointStateOffset;

   private zmq.SocketBase variablePublisher;
   
   public YoVariableProducer(int port, int numberOfVariables, int numberOfJointStates, ConcurrentRingBuffer<FullStateBuffer> mainBuffer,
         Collection<ConcurrentRingBuffer<RegistryBuffer>> buffers)
   {
      super("YoVariableProducer_" + port);
      this.port = port;
      this.mainBuffer = mainBuffer;
      this.buffers = buffers.toArray(new ConcurrentRingBuffer[buffers.size()]);
      
      this.jointStateOffset = numberOfVariables;
      int bufferSize = (1 + jointStateOffset + numberOfJointStates) * 8;
      
      backingArray = new byte[bufferSize];
      writeBuffer = ByteBuffer.wrap(backingArray).asLongBuffer();
   }
   
   private void updateBuffers(long timestamp)
   {
      for(int i = 0; i < buffers.length; i++)
      {
         ConcurrentRingBuffer<RegistryBuffer> buffer = buffers[i];
         if(buffer.poll())
         {
            RegistryBuffer newBuffer = buffer.peek();
            if(newBuffer != null && newBuffer.getTimestamp() < timestamp)
            {
               newBuffer = buffer.read();
               newBuffer.getIntoBuffer(writeBuffer, 1);
               buffer.flush();
            }
         }
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
      
      
      int bufferIndex = 0;
      byte[][] messageBuffers = new byte[NUMBER_OF_WRITE_BUFFERS][];
      zmq.Msg messages[] = new zmq.Msg[NUMBER_OF_WRITE_BUFFERS];
      for(int i = 0; i < NUMBER_OF_WRITE_BUFFERS; i++)
      {
         messageBuffers[i] = new byte[backingArray.length];
         messages[i] = new zmq.Msg(messageBuffers[i]);
      }
      
      while (true)
      {
         if(mainBuffer.poll())
         {
            FullStateBuffer fullStateBuffer;
   
            while((fullStateBuffer = mainBuffer.read()) != null)
            {
               
               writeBuffer.put(0, fullStateBuffer.getTimestamp());
               fullStateBuffer.getIntoBuffer(writeBuffer, 1);
               fullStateBuffer.getJointStatesInBuffer(writeBuffer, jointStateOffset + 1);
               updateBuffers(fullStateBuffer.getTimestamp());
               
               byte[] messageBuffer = messageBuffers[bufferIndex % NUMBER_OF_WRITE_BUFFERS];
               zmq.Msg msg = messages[bufferIndex % NUMBER_OF_WRITE_BUFFERS];
               
               System.arraycopy(backingArray, 0, messageBuffer, 0, backingArray.length);
               variablePublisher.send(msg, 0);
               
               bufferIndex++;
            }
            mainBuffer.flush();
         }
         try
         {
            Thread.sleep(1);
         }
         catch (InterruptedException e)
         {
            // Allow calling Thread.interrupt from YoVariableServer
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
      ctx.terminate();
   }
}