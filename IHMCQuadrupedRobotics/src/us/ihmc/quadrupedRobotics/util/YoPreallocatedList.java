package us.ihmc.quadrupedRobotics.util;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;

import java.util.ArrayList;

public class YoPreallocatedList<E> extends PreallocatedList<E>
{
   public interface DefaultElementFactory<E>
   {
      E createDefaultElement(String prefix, YoVariableRegistry registry);
   }

   public YoPreallocatedList(final String prefix, final YoVariableRegistry registry, int capacity,
         final DefaultElementFactory<E> defaultElementFactory)
   {
      super(new ArrayList<E>());
      for (int i = 0; i < capacity; i++)
      {
         super.elements.add(defaultElementFactory.createDefaultElement(prefix + "ListElement" + i, registry));
         super.indexes.add(new YoIntegerWrapper(new IntegerYoVariable(prefix + "ListIndex" + i, registry)));
         super.indexes.get(i).setValue(i);
      }
      super.size = new YoIntegerWrapper(new IntegerYoVariable(prefix + "ListSize", registry));
   }

   public YoPreallocatedList(final String prefix, final YoVariableRegistry registry, ArrayList<E> elements)
   {
      super(new ArrayList<E>());
      for (int i = 0; i < elements.size(); i++)
      {
         super.elements.add(elements.get(i));
         super.indexes.add(new YoIntegerWrapper(new IntegerYoVariable(prefix + "ListIndex" + i, registry)));
         super.indexes.get(i).setValue(i);
      }
      super.size = new YoIntegerWrapper(new IntegerYoVariable(prefix + "ListSize", registry));
      super.size.setValue(elements.size());
   }

   protected class YoIntegerWrapper extends IntegerWrapper
   {
      private final IntegerYoVariable yoValue;

      public YoIntegerWrapper(IntegerYoVariable yoValue)
      {
         super(yoValue.getIntegerValue());
         this.yoValue = yoValue;
      }

      @Override
      public int getValue()
      {
         return yoValue.getIntegerValue();
      }

      @Override
      public void setValue(int value)
      {
         yoValue.set(value);
      }

      @Override
      public void increment()
      {
         yoValue.increment();
      }

      @Override
      public void decrement()
      {
         yoValue.decrement();
      }
   }
}
