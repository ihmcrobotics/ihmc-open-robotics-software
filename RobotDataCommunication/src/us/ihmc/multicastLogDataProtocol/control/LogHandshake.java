package us.ihmc.multicastLogDataProtocol.control;


// Kryo is only temporary, re-use handshake protobuf for generic improvements to protocol 
public class LogHandshake
{
   public byte[] protoShake;

   public String modelLoaderClass = null;
   public String modelName;
   public byte[] model;
   public String[] resourceDirectories;
   public byte[] resourceZip;

}
