package us.ihmc.robotDataCommunication;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.LongBuffer;
import java.util.Collection;
import java.util.zip.CRC32;

import org.zeromq.ZMQ;

import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.utilities.compression.SnappyUtils;

public class YoVariableProducer extends Thread
{
   private static final int NUMBER_OF_WRITE_BUFFERS = 128;

   private final zmq.Ctx ctx = zmq.ZMQ.zmq_init(1);

   private final int port;
   private final ConcurrentRingBuffer<FullStateBuffer> mainBuffer;
   private final ConcurrentRingBuffer<RegistryBuffer>[] buffers;

   private final byte[] compressedBackingArray;
   private final ByteBuffer byteWriteBuffer;
   private final LongBuffer writeBuffer;
   private final ByteBuffer compressedBuffer;

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

      byteWriteBuffer = ByteBuffer.allocate(bufferSize);
      writeBuffer = byteWriteBuffer.asLongBuffer();

      compressedBackingArray = new byte[SnappyUtils.maxCompressedLength(bufferSize) + 4];
      compressedBuffer = ByteBuffer.wrap(compressedBackingArray);
   }

   private void updateBuffers(long timestamp)
   {
      for (int i = 0; i < buffers.length; i++)
      {
         ConcurrentRingBuffer<RegistryBuffer> buffer = buffers[i];
         if (buffer.poll())
         {
            RegistryBuffer newBuffer = buffer.peek();
            if (newBuffer != null && newBuffer.getTimestamp() < timestamp)
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
      RemoteVisualizationUtils.mayRaise(variablePublisher);

      if (!variablePublisher.bind("tcp://*:" + port))
      {
         RemoteVisualizationUtils.mayRaise(variablePublisher);
      }

      int bufferIndex = 0;
      byte[][] messageBuffers = new byte[NUMBER_OF_WRITE_BUFFERS][];
      zmq.Msg messages[] = new zmq.Msg[NUMBER_OF_WRITE_BUFFERS];
      for (int i = 0; i < NUMBER_OF_WRITE_BUFFERS; i++)
      {
         messageBuffers[i] = new byte[compressedBackingArray.length];
         messages[i] = new zmq.Msg(messageBuffers[i]);
      }

      CRC32 crc32 = new CRC32();

      while (true)
      {
         if (mainBuffer.poll())
         {
            FullStateBuffer fullStateBuffer;

            while ((fullStateBuffer = mainBuffer.read()) != null)
            {

               writeBuffer.put(0, fullStateBuffer.getTimestamp());
               fullStateBuffer.getIntoBuffer(writeBuffer, 1);
               fullStateBuffer.getJointStatesInBuffer(writeBuffer, jointStateOffset + 1);
               updateBuffers(fullStateBuffer.getTimestamp());

               byte[] messageBuffer = messageBuffers[bufferIndex % NUMBER_OF_WRITE_BUFFERS];
               zmq.Msg msg = messages[bufferIndex % NUMBER_OF_WRITE_BUFFERS];

               byteWriteBuffer.clear();
               compressedBuffer.clear();
               compressedBuffer.position(4);
               try
               {
                  SnappyUtils.compress(byteWriteBuffer, compressedBuffer);
                  compressedBuffer.flip();
                  compressedBuffer.position(4);
               }
               catch (IllegalArgumentException | IOException e)
               {
                  e.printStackTrace();
                  continue;
               }

               crc32.reset();
               crc32.update(compressedBackingArray, compressedBuffer.position(), compressedBuffer.remaining());
               compressedBuffer.putInt(0, (int) crc32.getValue());
               
               msg.setSize(compressedBuffer.limit());
               System.arraycopy(compressedBackingArray, 0, messageBuffer, 0, compressedBuffer.limit());
               
               
               if(!variablePublisher.send(msg, 0))
               {
                  System.err.println("Cannot send messages " + variablePublisher.errno());
               }

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

      if (variablePublisher.check_tag())
         variablePublisher.close(); // don't close if already closed
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