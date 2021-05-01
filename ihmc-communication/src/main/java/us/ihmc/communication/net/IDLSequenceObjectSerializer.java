package us.ihmc.communication.net;

import java.util.HashMap;
import java.util.Map;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Registration;
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

   private final Map<Class, TopicDataType> topicDataTypes = new HashMap<>();

   public IDLSequenceObjectSerializer()
   {
      setImmutable(true);
   }

   /** @see #setElementClass(Class, Serializer) */
   public IDLSequenceObjectSerializer(Class elementClass, Serializer serializer)
   {
      this();
      setElementClass(elementClass, serializer);
   }

   /**
    * @param elementClass The concrete class of each element. This saves 1-2 bytes per element. Set
    *           to null if the class is not known or varies per element (default).
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
      kryo.writeClass(output, collection.getTopicDataType().getClass());

      Serializer serializer = this.serializer;

      if (genericType != null)
      {
         if (serializer == null)
            serializer = kryo.getSerializer(genericType);
         genericType = null;
      }

      if (length == 0)
         return;

      if (serializer == null)
         serializer = kryo.getSerializer(collection.get(0).getClass());

      if (serializer == null)
         serializer = kryo.getDefaultSerializer(collection.get(0).getClass());
      

//      if (serializer != null)
//      {
//         for (int i = 0; i < collection.size(); i++)
//            kryo.writeObject(output, collection.get(i), serializer);
//      }
//      else
      {
         for (int i = 0; i < collection.size(); i++)
            kryo.writeClassAndObject(output, collection.get(i));
      }
   }

   @SuppressWarnings("unchecked")
   @Override
   public IDLSequence.Object read(Kryo kryo, Input input, Class<? extends IDLSequence.Object> type)
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
      Registration topicDataTypeRegistration = kryo.readClass(input);

      Class topicDataTypeClass = topicDataTypeRegistration.getType();
      TopicDataType topicDataType = topicDataTypes.get(topicDataTypeClass);

      if (topicDataType == null)
      {
         try
         {
            topicDataType = (TopicDataType) topicDataTypeClass.newInstance();
         }
         catch (InstantiationException | IllegalAccessException e)
         {
            e.printStackTrace();
            return null;
         }
         topicDataTypes.put(topicDataTypeClass, topicDataType);
      }

      IDLSequence.Object resultList = new IDLSequence.Object(length, null, topicDataType);

//      if (serializer != null)
//      {
//         for (int i = 0; i < length; i++)
//            topicDataType.copy(kryo.readObject(input, elementClass, serializer), resultList.add());
//      }
//      else
      {
         for (int i = 0; i < length; i++)
            topicDataType.copy(kryo.readClassAndObject(input), resultList.add());
      }

      kryo.reference(resultList);

      return resultList;
   }
}
