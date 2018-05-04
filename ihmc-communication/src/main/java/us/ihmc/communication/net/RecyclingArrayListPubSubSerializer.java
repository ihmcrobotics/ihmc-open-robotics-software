package us.ihmc.communication.net;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import com.esotericsoftware.kryo.Kryo;
import com.esotericsoftware.kryo.Serializer;
import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryo.io.Output;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.idl.RecyclingArrayListPubSub;

@SuppressWarnings("rawtypes")
public class RecyclingArrayListPubSubSerializer extends Serializer<RecyclingArrayListPubSub>
{
   private boolean elementsCanBeNull = true;
   private Serializer serializer;
   private Class elementClass;
   private Class genericType;

   public RecyclingArrayListPubSubSerializer()
   {
   }

   /** @see #setElementClass(Class, Serializer) */
   public RecyclingArrayListPubSubSerializer(Class elementClass, Serializer serializer)
   {
      setElementClass(elementClass, serializer);
   }

   /**
    * @see #setElementClass(Class, Serializer)
    * @see #setElementsCanBeNull(boolean)
    */
   public RecyclingArrayListPubSubSerializer(Class elementClass, Serializer serializer, boolean elementsCanBeNull)
   {
      setElementClass(elementClass, serializer);
      this.elementsCanBeNull = elementsCanBeNull;
   }

   /**
    * @param elementsCanBeNull False if all elements are not null. This saves 1 byte per element if
    *           elementClass is set. True if it is not known (default).
    */
   public void setElementsCanBeNull(boolean elementsCanBeNull)
   {
      this.elementsCanBeNull = elementsCanBeNull;
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

   public void write(Kryo kryo, Output output, RecyclingArrayListPubSub collection)
   {
      int length = collection.size();
      output.writeInt(length, true);
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
         if (elementsCanBeNull)
         {
            for (Object element : collection)
               kryo.writeObjectOrNull(output, element, serializer);
         }
         else
         {
            for (Object element : collection)
               kryo.writeObject(output, element, serializer);
         }
      }
      else
      {
         for (Object element : collection)
            kryo.writeClassAndObject(output, element);
      }
   }

   /**
    * Used by {@link #read(Kryo, Input, Class)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   @SuppressWarnings("unchecked")
   protected RecyclingArrayListPubSub create(Kryo kryo, Input input, Class<RecyclingArrayListPubSub> type)
   {
      return new RecyclingArrayListPubSub(elementClass, () -> kryo.newInstance(elementClass), 0);
   }

   @SuppressWarnings("unchecked")
   public RecyclingArrayListPubSub read(Kryo kryo, Input input, Class<RecyclingArrayListPubSub> type)
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
      RecyclingArrayListPubSub resultList;

      if (length == 0)
      {
         resultList = new RecyclingArrayListPubSub<>();
      }
      else
      {
         List tempList = new ArrayList(length);
         
         if (serializer != null)
         {
            if (elementsCanBeNull)
            {
               for (int i = 0; i < length; i++)
                  tempList.add(kryo.readObjectOrNull(input, elementClass, serializer));
            }
            else
            {
               for (int i = 0; i < length; i++)
                  tempList.add(kryo.readObject(input, elementClass, serializer));
            }
         }
         else
         {
            for (int i = 0; i < length; i++)
               tempList.add(kryo.readClassAndObject(input));
         }
         
         Class elementClass = tempList.get(0).getClass();
         resultList = new RecyclingArrayListPubSub(elementClass, () -> kryo.newInstance(elementClass), length);
         for (int i = 0; i < length; i++)
            addElementToList(resultList, tempList.get(i), kryo);
      }

      kryo.reference(resultList);
      
      
      return resultList;
   }

   /**
    * Used by {@link #copy(Kryo, Collection)} to create the new object. This can be overridden to
    * customize object creation, eg to call a constructor with arguments. The default implementation
    * uses {@link Kryo#newInstance(Class)}.
    */
   protected RecyclingArrayListPubSub createCopy(Kryo kryo, RecyclingArrayListPubSub original)
   {
      return kryo.newInstance(original.getClass());
   }

   @SuppressWarnings("unchecked")
   public RecyclingArrayListPubSub copy(Kryo kryo, RecyclingArrayListPubSub original)
   {
      RecyclingArrayListPubSub copy;
      if (original.isEmpty())
      {
         return createCopy(kryo, original);
      }
      else
      {
         Class elementClass = original.get(0).getClass();
         copy = new RecyclingArrayListPubSub(elementClass, () -> kryo.newInstance(elementClass), original.size());
      }

      kryo.reference(copy);

      for (Object element : original)
      {
         addElementToList(copy, element, kryo);
      }
      return copy;
   }

   @SuppressWarnings("unchecked")
   private void addElementToList(RecyclingArrayListPubSub list, Object element, Kryo kryo)
   {
      if (element instanceof Settable)
      {
         ((Settable) list.add()).set(kryo.copy(element));
      }
      else if (element instanceof StringBuilder)
      {
         StringBuilder elementDestination = (StringBuilder) list.add();
         elementDestination.append((StringBuilder) kryo.copy(element));
      }
      else
      {
         throw new UnsupportedOperationException("Unsupported type: " + element.getClass().getSimpleName());
      }
   }
}
