package us.ihmc.atlas.operatorInterfaceDebugging;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.KryoStreamDeSerializer;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.io.files.FileTools;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.UnitConversions;

public class AtlasUIPacketLoader
{
   private static final double PLAYBACK_SPEED = 2.0;
   
   public AtlasUIPacketLoader() throws IOException
   {
      
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      
      final KryoStreamDeSerializer kryoStreamDeSerializer = new KryoStreamDeSerializer(UnitConversions.megabytesToBytes(500));
      kryoStreamDeSerializer.registerClasses(netClassList);
      
      final PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, UnitConversions.megabytesToBytes(500), UnitConversions.megabytesToBytes(500), netClassList);
      packetCommunicator.connect();
      packetCommunicator.attachStateListener(new NetStateListener()
      {
         private DataInputStream fileDataInputStream;
         private BufferedReader timingReader;
         
         public void openFileDataStream() throws IOException
         {
            fileDataInputStream = FileTools.getFileDataInputStream(AtlasUIPacketRecorder.getPacketRecordingFilePath(), UnitConversions.megabytesToBytes(500));
            timingReader = FileTools.newBufferedReader(AtlasUIPacketRecorder.getPacketTimingPath());
         }
         
         @Override
         public void disconnected()
         {
         }
         
         @Override
         public void connected()
         {
            PrintTools.info("Connected. Press Enter to send...");
            
            try
            {
//               System.in.read();
               
               openFileDataStream();
               
               Packet<?> packet = null;
               do
               {
                  packet = (Packet<?>) kryoStreamDeSerializer.read(fileDataInputStream);
                  PrintTools.info("Sending: " + packet);

                  packetCommunicator.send(packet);
                  
                  double timeToWait = 0.01;
                  if (timingReader.ready())
                     timeToWait = Double.valueOf(timingReader.readLine()) / PLAYBACK_SPEED;
                  
                  ThreadTools.sleepSeconds(timeToWait);
                  
                  if (fileDataInputStream.available() < 1)
                  {
                     fileDataInputStream.close();
                     
                     ThreadTools.sleepSeconds(0.5);
                     
                     openFileDataStream();
                  }
               }
               while (packet != null && fileDataInputStream.available() > 1);

               fileDataInputStream.close();
               
               packetCommunicator.close();
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
      });
   }
   
   public static void main(String[] args) throws IOException
   {
      new AtlasUIPacketLoader();
   }
}
