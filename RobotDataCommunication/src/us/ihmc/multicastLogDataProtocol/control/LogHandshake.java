package us.ihmc.multicastLogDataProtocol.control;


// Kryo is only temporary, re-use handshake protobuf for generic improvements to protocol 
public class LogHandshake
{
   public byte[] protoShake;
   public byte[] model;
}
