package us.ihmc.communication.net;

import java.util.Collection;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.idl.IDLSequence;

@SuppressWarnings("rawtypes")
public class IDLSequenceIntegerSerializer extends Serializer<IDLSequence.Integer>
{
   private static final String TYPE_CODE = "type_3";

   public IDLSequenceIntegerSerializer()
   {
   }

   public void setGenerics(Kryo kryo, Class[] generics)
   {
   }

   public void write(Kryo kryo, Output output, IDLSequence.Integer collection)
   {
      int length = collection.size();
      output.writeInt(length, true);

      for (int i = 0; i < length; i++)
         output.writeInt(collection.get(i));
   }

   public IDLSequence.Integer read(Kryo kryo, Input input, Class<? extends IDLSequence.Integer> type)
   {
      int length = input.readInt(true);
      IDLSequence.Integer resultList = new IDLSequence.Integer(length, TYPE_CODE);

      for (int i = 0; i < length; i++)
         resultList.add(input.readInt());

      kryo.reference(resultList);

      return resultList;
   }

   /**
    * Used by {@link #copy(Kryo, Collection)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   protected IDLSequence.Integer createCopy(Kryo kryo, IDLSequence.Integer original)
   {
      return kryo.newInstance(original.getClass());
   }

   public IDLSequence.Integer copy(Kryo kryo, IDLSequence.Integer original)
   {
      IDLSequence.Integer copy = new IDLSequence.Integer(original.size(), TYPE_CODE);
      copy.set(original);
      return copy;
   }
}
