package us.ihmc.robotDataLogger.rtps;

import java.nio.ByteBuffer;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Base64;

import us.ihmc.idl.CDR;
import us.ihmc.robotDataLogger.Handshake;
import us.ihmc.robotDataLogger.HandshakePubSubType;

public class HandshakeHashCalculator extends CDR
{
   
   private final MessageDigest md;
   private final ByteBuffer bytes = ByteBuffer.allocate(8);
   
   public static String calculateHash(Handshake handshake)
   {
      HandshakeHashCalculator calculator = new HandshakeHashCalculator();
      HandshakePubSubType.write(handshake, calculator);

      return calculator.getHash();
   }

   private HandshakeHashCalculator()
   {
      try
      {
         md = MessageDigest.getInstance("SHA-256");
      }
      catch (NoSuchAlgorithmException e)
      {
         throw new RuntimeException(e);
      }

   }

   private String getHash()
   {
      return Base64.getEncoder().encodeToString(md.digest());
   }
   
   private void flipClearAndUpdate()
   {
      bytes.flip();
      md.update(bytes);
      bytes.clear();
   }
   
   public void write_type_1(short val)
   {
      bytes.putShort(val);
      flipClearAndUpdate();
   }
   
   public void write_type_2(int val)
   {
      bytes.putInt(val);
      flipClearAndUpdate();
   }
   
   public void write_type_5(float val)
   {
      bytes.putFloat(val);
      flipClearAndUpdate();
   }
   
   public void write_type_6(double val)
   {
      bytes.putDouble(val);
      flipClearAndUpdate();
   }
   
   public void write_type_8(char val)
   {
      bytes.putChar(val);
      flipClearAndUpdate();
   }
   
   public void write_type_9(byte val)
   {
      bytes.put(val);
      flipClearAndUpdate();
   }
   
   public void write_type_d(StringBuilder str)
   {
      write_type_2(str.length() + 1);
      for (int i = 0; i < str.length(); i++)
      {
         bytes.put((byte) str.charAt(i));
         flipClearAndUpdate();
      }
   }
   
   public void write_type_11(long val)
   {
      bytes.putLong(val);
      flipClearAndUpdate();
   }
   
   public void write_type_15(StringBuilder str)
   {
      write_type_2(str.length());
      for (int i = 0; i < str.length(); i++)
      {
         bytes.putChar(str.charAt(i));
         flipClearAndUpdate();
      }
   }

}
