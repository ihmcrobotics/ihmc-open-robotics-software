package us.ihmc.atlas.operatorInterfaceDebugging;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.util.Scanner;

import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.communication.net.KryoStreamDeSerializer;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.tools.thread.ThreadTools;

public class AtlasUIPacketLoader
{
   private static final boolean PRESS_ENTER = false;
   private static final boolean LOOP_PLAYBACK = false;
   private static final double PLAYBACK_SPEED = 0.5;

   public AtlasUIPacketLoader() throws IOException
   {

      IHMCCommunicationKryoNetClassList netClassList = new IHMCCommunicationKryoNetClassList();

      final KryoStreamDeSerializer kryoStreamDeSerializer = new KryoStreamDeSerializer(Conversions.megabytesToBytes(500));
      kryoStreamDeSerializer.registerClasses(netClassList);

      final PacketCommunicator packetCommunicator = PacketCommunicator.createTCPPacketCommunicatorServer(NetworkPorts.NETWORK_PROCESSOR_TO_UI_TCP_PORT, Conversions.megabytesToBytes(500), Conversions.megabytesToBytes(500), netClassList);
      packetCommunicator.connect();
      packetCommunicator.attachStateListener(new NetStateListener()
      {
         private DataInputStream fileDataInputStream;
         private BufferedReader timingReader;

         public void openFileDataStream() throws IOException
         {
            fileDataInputStream = FileTools.newFileDataInputStream(AtlasUIPacketRecorder.getPacketRecordingFilePath(), Conversions.megabytesToBytes(500));
            timingReader = Files.newBufferedReader(AtlasUIPacketRecorder.getPacketTimingPath());
         }

         @Override
         public void disconnected()
         {
         }

         @Override
         public void connected()
         {
            if (PRESS_ENTER)
               PrintTools.info("Connected. Press Enter to send...");
            else
               PrintTools.info("Connected");

            try
            {
               if (PRESS_ENTER)
               {
                  Scanner scanner = new Scanner(System.in);
                  scanner.nextLine();
                  scanner.close();
               }

               openFileDataStream();

               Packet<?> packet = null;

               do
               {
                  try
                  {
                     packet = (Packet<?>) kryoStreamDeSerializer.read(fileDataInputStream);
                     PrintTools.info("Sending: " + packet);

                     packetCommunicator.send(packet);

                     double timeToWait = 0.01;
                     if (timingReader.ready())
                        timeToWait = Double.valueOf(timingReader.readLine()) / PLAYBACK_SPEED;

                     ThreadTools.sleepSeconds(timeToWait);
                  }
                  catch (IOException e)
                  {
                     PrintTools.error(e.getMessage());
                  }

                  if (LOOP_PLAYBACK)
                  {
                     if (fileDataInputStream.available() < 1)
                     {
                        fileDataInputStream.close();

                        ThreadTools.sleepSeconds(0.5);

                        openFileDataStream();
                     }
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
