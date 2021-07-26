package us.ihmc.communication.net;

import java.util.Collection;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.idl.IDLSequence;

@SuppressWarnings("rawtypes")
public class IDLSequenceDoubleSerializer extends Serializer<IDLSequence.Double>
{
   private static final String TYPE_CODE = "type_6";

   public IDLSequenceDoubleSerializer()
   {
   }

   public void setGenerics(Kryo kryo, Class[] generics)
   {
   }

   public void write(Kryo kryo, Output output, IDLSequence.Double collection)
   {
      int length = collection.size();
      output.writeInt(length, true);

      for (int i = 0; i < length; i++)
         output.writeDouble(collection.get(i));
   }

   public IDLSequence.Double read(Kryo kryo, Input input, Class<? extends IDLSequence.Double> type)
   {
      int length = input.readInt(true);
      IDLSequence.Double resultList = new IDLSequence.Double(length, TYPE_CODE);

      for (int i = 0; i < length; i++)
         resultList.add(input.readDouble());

      kryo.reference(resultList);

      return resultList;
   }

   /**
    * Used by {@link #copy(Kryo, Collection)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   protected IDLSequence.Double createCopy(Kryo kryo, IDLSequence.Double original)
   {
      return kryo.newInstance(original.getClass());
   }

   public IDLSequence.Double copy(Kryo kryo, IDLSequence.Double original)
   {
      IDLSequence.Double copy = new IDLSequence.Double(original.size(), TYPE_CODE);
      copy.set(original);
      return copy;
   }
}
