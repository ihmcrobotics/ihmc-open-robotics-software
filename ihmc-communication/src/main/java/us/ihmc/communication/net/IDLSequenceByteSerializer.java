package us.ihmc.communication.net;

import java.util.Collection;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.idl.IDLSequence;

@SuppressWarnings("rawtypes")
public class IDLSequenceByteSerializer extends Serializer<IDLSequence.Byte>
{
   private static final String TYPE_CODE = "type_9";

   public IDLSequenceByteSerializer()
   {
   }

   public void setGenerics(Kryo kryo, Class[] generics)
   {
   }

   public void write(Kryo kryo, Output output, IDLSequence.Byte collection)
   {
      int length = collection.size();
      output.writeInt(length, true);

      for (int i = 0; i < length; i++)
         output.writeByte(collection.get(i));
   }

   public IDLSequence.Byte read(Kryo kryo, Input input, Class<? extends IDLSequence.Byte> type)
   {
      int length = input.readInt(true);
      IDLSequence.Byte resultList = new IDLSequence.Byte(length, TYPE_CODE);

      for (int i = 0; i < length; i++)
         resultList.add(input.readByte());

      kryo.reference(resultList);

      return resultList;
   }

   /**
    * Used by {@link #copy(Kryo, Collection)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   protected IDLSequence.Byte createCopy(Kryo kryo, IDLSequence.Byte original)
   {
      return kryo.newInstance(original.getClass());
   }

   public IDLSequence.Byte copy(Kryo kryo, IDLSequence.Byte original)
   {
      IDLSequence.Byte copy = new IDLSequence.Byte(original.size(), TYPE_CODE);
      copy.set(original);
      return copy;
   }
}
