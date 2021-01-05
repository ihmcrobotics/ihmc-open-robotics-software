package us.ihmc.communication.net;

import java.util.Collection;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.idl.IDLSequence;

@SuppressWarnings("rawtypes")
public class IDLSequenceBooleanSerializer extends Serializer<IDLSequence.Boolean>
{
   private static final String TYPE_CODE = "type_3";

   public IDLSequenceBooleanSerializer()
   {
   }

   public void setGenerics(Kryo kryo, Class[] generics)
   {
   }

   public void write(Kryo kryo, Output output, IDLSequence.Boolean collection)
   {
      int length = collection.size();
      output.writeInt(length, true);

      for (int i = 0; i < length; i++)
         output.writeBoolean(collection.getBoolean(i));
   }

   @Override
   public IDLSequence.Boolean read(Kryo kryo, Input input, Class<? extends IDLSequence.Boolean> type)
   {
      int length = input.readInt(true);
      IDLSequence.Boolean resultList = new IDLSequence.Boolean(length, TYPE_CODE);

      for (int i = 0; i < length; i++)
         resultList.add(input.readBoolean());

      kryo.reference(resultList);

      return resultList;
   }

   /**
    * Used by {@link copy(Kryo, Collection)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   protected IDLSequence.Boolean createCopy(Kryo kryo, IDLSequence.Boolean original)
   {
      return kryo.newInstance(original.getClass());
   }

   public IDLSequence.Boolean copy(Kryo kryo, IDLSequence.Boolean original)
   {
      IDLSequence.Boolean copy = new IDLSequence.Boolean(original.size(), TYPE_CODE);
      copy.set(original);
      return copy;
   }
}
