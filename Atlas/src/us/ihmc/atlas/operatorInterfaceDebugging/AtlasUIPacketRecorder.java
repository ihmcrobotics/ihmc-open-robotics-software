package us.ihmc.atlas.operatorInterfaceDebugging;

import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.KryoStreamSerializer;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.utilities.DateTools;
import us.ihmc.utilities.io.files.FileTools;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.UnitConversions;

public class AtlasUIPacketRecorder
{
   private static final Path PACKET_RECORDINGS_PATH = Paths.get("./packetRecordings");

   public AtlasUIPacketRecorder() throws IOException
   {
      final DataOutputStream fileDataOutputStream = FileTools.getFileDataOutputStream(getPacketRecordingFilePath());
      
      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();
      
      final KryoStreamSerializer kryoStreamSerializer = new KryoStreamSerializer(UnitConversions.megabytesToBytes(10));
      kryoStreamSerializer.registerClasses(netClassList);
      
      PacketCommunicator packetClient = PacketCommunicator.createTCPPacketCommunicatorClient("localhost", NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, netClassList);
      packetClient.attachStateListener(new NetStateListener()
      {
         @Override
         public void disconnected()
         {
            PrintTools.info("Disconnected");
            try
            {
               fileDataOutputStream.flush();
               fileDataOutputStream.close();
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
         
         @Override
         public void connected()
         {
            PrintTools.info("Connected");
         }
      });
      packetClient.connect();
      packetClient.attachGlobalListener(new GlobalPacketConsumer()
      {
         @Override
         public void receivedPacket(Packet<?> packet)
         {
            try
            {
               PrintTools.info("Receiving packet: " + packet);
               kryoStreamSerializer.write(fileDataOutputStream, packet);
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
      });
      
      System.in.read();
      
      packetClient.close();
   }
   
   public static Path getPacketRecordingFilePath()
   {
      FileTools.ensureDirectoryExists(PACKET_RECORDINGS_PATH);
      return PACKET_RECORDINGS_PATH.resolve("PacketRecording_" + DateTools.getDateString() + "1");
   }
   
   public static void main(String[] args) throws IOException
   {
      new AtlasUIPacketRecorder();
   }
}
