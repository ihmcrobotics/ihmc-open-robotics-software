package us.ihmc.communication.net;

import java.util.Collection;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.idl.IDLSequence;
import us.ihmc.pubsub.TopicDataType;

@SuppressWarnings("rawtypes")
public class IDLSequenceObjectSerializer extends Serializer<IDLSequence.Object>
{
   private Serializer serializer;
   private Class elementClass;
   private Class genericType;

   public IDLSequenceObjectSerializer()
   {
   }

   /** @see #setElementClass(Class, Serializer) */
   public IDLSequenceObjectSerializer(Class elementClass, Serializer serializer)
   {
      setElementClass(elementClass, serializer);
   }

   /**
    * @param elementClass The concrete class of each element. This saves 1-2 bytes per element. Set to
    *           null if the class is not known or varies per element (default).
    * @param serializer The serializer to use for each element.
    */
   public void setElementClass(Class elementClass, Serializer serializer)
   {
      this.elementClass = elementClass;
      this.serializer = serializer;
   }

   public void setGenerics(Kryo kryo, Class[] generics)
   {
      if (kryo.isFinal(generics[0]))
         genericType = generics[0];
   }

   public void write(Kryo kryo, Output output, IDLSequence.Object collection)
   {
      int length = collection.size();
      output.writeInt(length, true);
      TopicDataType topicDataType = collection.getTopicDataType().newInstance();
      kryo.writeClassAndObject(output, topicDataType);
      Serializer serializer = this.serializer;
      if (genericType != null)
      {
         if (serializer == null)
            serializer = kryo.getSerializer(genericType);
         genericType = null;
      }

      if (length == 0)
         return;

      if (serializer != null)
      {
         for (int i = 0; i < collection.size(); i++)
            kryo.writeObject(output, collection.get(i), serializer);
      }
      else
      {
         for (int i = 0; i < collection.size(); i++)
            kryo.writeClassAndObject(output, collection.get(i));
      }
   }

   @SuppressWarnings("unchecked")
   public IDLSequence.Object read(Kryo kryo, Input input, Class<IDLSequence.Object> type)
   {
      if (genericType != null)
      {
         if (serializer == null)
         {
            elementClass = genericType;
            serializer = kryo.getSerializer(genericType);
         }
         genericType = null;
      }

      int length = input.readInt(true);
      TopicDataType topicDataType = (TopicDataType) kryo.readClassAndObject(input);
      IDLSequence.Object resultList = new IDLSequence.Object(length, topicDataType.createData().getClass(), topicDataType);

      if (serializer != null)
      {
         for (int i = 0; i < length; i++)
            topicDataType.copy(kryo.readObject(input, elementClass, serializer), resultList.add());
      }
      else
      {
         for (int i = 0; i < length; i++)
            topicDataType.copy(kryo.readClassAndObject(input), resultList.add());
      }

      kryo.reference(resultList);

      return resultList;
   }

   /**
    * Used by {@link #copy(Kryo, Collection)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   protected IDLSequence.Object createCopy(Kryo kryo, IDLSequence.Object original)
   {
      return kryo.newInstance(original.getClass());
   }

   @SuppressWarnings("unchecked")
   public IDLSequence.Object copy(Kryo kryo, IDLSequence.Object original)
   {
      TopicDataType topicDataType = original.getTopicDataType();
      Class clazz = topicDataType.createData().getClass();
      IDLSequence.Object copy = new IDLSequence.Object(original.size(), clazz, topicDataType);
      copy.set(original);
      return copy;
   }
}
