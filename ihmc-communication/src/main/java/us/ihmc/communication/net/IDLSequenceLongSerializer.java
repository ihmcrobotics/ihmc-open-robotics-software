package us.ihmc.communication.net;

import java.util.Collection;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.idl.IDLSequence;

@SuppressWarnings("rawtypes")
public class IDLSequenceLongSerializer extends Serializer<IDLSequence.Long>
{
   private static final String TYPE_CODE = "type_4";

   public IDLSequenceLongSerializer()
   {
   }

   public void setGenerics(Kryo kryo, Class[] generics)
   {
   }

   public void write(Kryo kryo, Output output, IDLSequence.Long collection)
   {
      int length = collection.size();
      output.writeInt(length, true);

      for (int i = 0; i < length; i++)
         output.writeLong(collection.get(i));
   }

   public IDLSequence.Long read(Kryo kryo, Input input, Class<? extends IDLSequence.Long> type)
   {
      int length = input.readInt(true);
      IDLSequence.Long resultList = new IDLSequence.Long(length, TYPE_CODE);

      for (int i = 0; i < length; i++)
         resultList.add(input.readLong());

      kryo.reference(resultList);

      return resultList;
   }

   /**
    * Used by {@link #copy(Kryo, Collection)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   protected IDLSequence.Long createCopy(Kryo kryo, IDLSequence.Long original)
   {
      return kryo.newInstance(original.getClass());
   }

   public IDLSequence.Long copy(Kryo kryo, IDLSequence.Long original)
   {
      IDLSequence.Long copy = new IDLSequence.Long(original.size(), TYPE_CODE);
      copy.set(original);
      return copy;
   }
}
