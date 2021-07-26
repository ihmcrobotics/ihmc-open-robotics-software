package us.ihmc.communication.net;

import java.util.Collection;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.idl.IDLSequence;

@SuppressWarnings("rawtypes")
public class IDLSequenceFloatSerializer extends Serializer<IDLSequence.Float>
{
   private static final String TYPE_CODE = "type_5";

   public IDLSequenceFloatSerializer()
   {
   }

   public void setGenerics(Kryo kryo, Class[] generics)
   {
   }

   public void write(Kryo kryo, Output output, IDLSequence.Float collection)
   {
      int length = collection.size();
      output.writeInt(length, true);

      for (int i = 0; i < length; i++)
         output.writeFloat(collection.get(i));
   }

   public IDLSequence.Float read(Kryo kryo, Input input, Class<? extends IDLSequence.Float> type)
   {
      int length = input.readInt(true);
      IDLSequence.Float resultList = new IDLSequence.Float(length, TYPE_CODE);

      for (int i = 0; i < length; i++)
         resultList.add(input.readFloat());

      kryo.reference(resultList);

      return resultList;
   }

   /**
    * Used by {@link #copy(Kryo, Collection)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   protected IDLSequence.Float createCopy(Kryo kryo, IDLSequence.Float original)
   {
      return kryo.newInstance(original.getClass());
   }

   public IDLSequence.Float copy(Kryo kryo, IDLSequence.Float original)
   {
      IDLSequence.Float copy = new IDLSequence.Float(original.size(), TYPE_CODE);
      copy.set(original);
      return copy;
   }
}
