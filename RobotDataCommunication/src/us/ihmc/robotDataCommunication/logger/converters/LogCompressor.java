package us.ihmc.robotDataCommunication.logger.converters;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;

import us.ihmc.robotDataCommunication.YoVariableHandshakeParser;
import us.ihmc.robotDataCommunication.logger.LogProperties;
import us.ihmc.robotDataCommunication.logger.LogPropertiesReader;
import us.ihmc.robotDataCommunication.logger.util.FileSelectionDialog;
import us.ihmc.utilities.compression.SnappyUtils;

public class LogCompressor
{
   public static void main(String[] args) throws IOException
   {
      File directory = FileSelectionDialog.loadDirectoryWithFileNamed("robotData.log");
      File log = new File(directory, "robotData.log");
      LogProperties properties = new LogPropertiesReader(log);

      if (!properties.getCompressed())
      {
         System.out.println("Compressing " + log);

         YoVariableHandshakeParser handshake = getHandshake(new File(directory, properties.getHandshakeFile()));
         int bufferSize = (1 + handshake.getNumberOfVariables() + handshake.getNumberOfJointStateVariables()) * 8;


         File logdata = new File(directory, properties.getVariableDataFile());
         if (!logdata.exists())
         {
            throw new RuntimeException("Cannot find " + properties.getVariableDataFile());
         }
         
         properties.setCompressed(true);
         properties.setVariableDataFile("robotData.bsz");
         properties.setVariablesIndexFile("robotData.dat");
         File compressedData = new File(directory, properties.getVariableDataFile());
         File indexData = new File(directory, properties.getVariablesIndexFile());
         
         ByteBuffer indexBuffer = ByteBuffer.allocate(8);
         ByteBuffer compressed = ByteBuffer.allocate(SnappyUtils.maxCompressedLength(bufferSize));
         ByteBuffer uncompressed = ByteBuffer.allocate(bufferSize);
         FileChannel logChannel = new FileInputStream(logdata).getChannel();
         
         FileChannel compressedChannel = new FileOutputStream(compressedData).getChannel();
         FileChannel indexChannel = new FileOutputStream(indexData).getChannel();
         
         int count = 0;
         while(logChannel.read(uncompressed) != -1)
         {
            if(count % 100 == 0)
            {
               System.out.print(".");               
               if(count % 8000 == 0)
               {
                  System.out.println();
               }
            }
            
            if(uncompressed.position() != uncompressed.limit())
            {
               throw new RuntimeException("Did not read full length segment");
            }
            
            uncompressed.flip();
            SnappyUtils.compress(uncompressed, compressed);
            compressed.flip();

            indexBuffer.putLong(compressedChannel.position());
            indexBuffer.flip();
            indexChannel.write(indexBuffer);
            
            compressedChannel.write(compressed);
            
            indexBuffer.clear();
            uncompressed.clear();
            compressed.clear();
            
            count++;
         }
         
         System.out.println("DONE");
         indexChannel.close();
         compressedChannel.close();
         logChannel.close();
         
         FileWriter writer = new FileWriter(log);
         properties.store(writer, "Converted by LogCompressor");
         writer.close();
      }
      else
      {
         System.err.println("Log file is already compressed");
      }
   }

   private static YoVariableHandshakeParser getHandshake(File handshake) throws IOException
   {
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + handshake);
      }

      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();

      YoVariableHandshakeParser parser = new YoVariableHandshakeParser("logged", true);
      parser.parseFrom(handshakeData);
      return parser;
   }
}
