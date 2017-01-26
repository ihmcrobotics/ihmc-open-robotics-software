package us.ihmc.robotDataLogger.logger.converters;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import us.ihmc.robotDataLogger.YoVariableHandshakeParser;

public class ConverterUtil
{

   static YoVariableHandshakeParser getHandshake(File handshake) throws IOException
   {
      if (!handshake.exists())
      {
         throw new RuntimeException("Cannot find " + handshake);
      }
   
      DataInputStream handshakeStream = new DataInputStream(new FileInputStream(handshake));
      byte[] handshakeData = new byte[(int) handshake.length()];
      handshakeStream.readFully(handshakeData);
      handshakeStream.close();
   
      YoVariableHandshakeParser parser = new YoVariableHandshakeParser("logged");
      parser.parseFrom(handshakeData);
      return parser;
   }

}
