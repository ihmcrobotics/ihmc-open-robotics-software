package us.ihmc.simulationConstructionSetTools.socketCommunication.commandrecorders;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

public class RecordedDataInputOutputStream
{
   private DataInputStream in;
   private DataOutputStream out;

   private StringBuffer buff = new StringBuffer();

   public RecordedDataInputOutputStream(DataInputStream in, DataOutputStream out)
   {
      this.in = in;
      this.out = out;
   }

   public void close() throws IOException
   {
      in.close();
   }

   public byte readByte() throws IOException
   {
      byte ret = in.readByte();

      buff.append("byte: ");
      buff.append(ret);
      buff.append("\n");

      return ret;
   }

   public int readInt() throws IOException
   {
      int ret = in.readInt();

      buff.append("int: ");
      buff.append(ret);
      buff.append("\n");

      return ret;
   }

   public String readUTF() throws IOException
   {
      String ret = in.readUTF();

      buff.append("String: ");
      buff.append(ret);
      buff.append("\n");

      return ret;
   }

   public float readFloat() throws IOException
   {
      float ret = in.readFloat();

      buff.append("float: ");
      buff.append(ret);
      buff.append("\n");

      return ret;
   }


   // Out Methods:

   public void flush() throws IOException
   {
      out.flush();
   }

   public void writeInt(int i) throws IOException
   {
      out.writeInt(i);
   }

   public void writeUTF(String s) throws IOException
   {
      out.writeUTF(s);
   }

   public void writeFloat(float f) throws IOException
   {
      out.writeFloat(f);
   }


   public void main(String[] args)
   {
      @SuppressWarnings("unused")
      RecordedDataInputOutputStream stream = new RecordedDataInputOutputStream(null, null);
   }

   public StringBuffer getBuffer()
   {
      return buff;
   }
}
