package us.ihmc.robotics.robotSide;

import us.ihmc.commons.lists.SupplierBuilder;

import java.lang.reflect.Array;
import java.util.function.Supplier;

@SuppressWarnings("unchecked")
public class RecyclingQuadrantDependentList<V> extends QuadrantDependentList<V>
{
   private final V[] elementStorageForWhenNull;
   private final Supplier<V> builder;
   private final V[][] valueArrays;
//   private final QuadrantDependentList<V> quadrantDependentList;

   public RecyclingQuadrantDependentList(Class<V> clazz)
   {
//      quadrantDependentList = new QuadrantDependentList<>();
      builder = SupplierBuilder.createFromEmptyConstructor(clazz);

      elementStorageForWhenNull = (V[]) Array.newInstance(clazz, 4);
      for (int i = 0; i < 4; i++)
      {
         V newInstance = builder.get();

         elementStorageForWhenNull[i] = newInstance;
      }

      valueArrays = (V[][]) new Object[5][];
      valueArrays[0] = (V[]) Array.newInstance(clazz, 0);
      valueArrays[1] = (V[]) Array.newInstance(clazz, 1);
      valueArrays[2] = (V[]) Array.newInstance(clazz, 2);
      valueArrays[3] = (V[]) Array.newInstance(clazz, 3);
      valueArrays[4] = (V[]) elementStorageForWhenNull;
   }

   public V add(RobotQuadrant robotQuadrant)
   {
      V element = elementStorageForWhenNull[robotQuadrant.ordinal()];
      // do nothing
      if (element == get(robotQuadrant))
      {
         return element;
      }
      // remove
      if (element == null && containsQuadrant(robotQuadrant))
      {
         super.set(robotQuadrant, element);
         fillValueArray();
         return element;
      }
      // add
      else if (element != null && !containsQuadrant(robotQuadrant))
      {
         V storageWhenNull = elementStorageForWhenNull[robotQuadrant.ordinal()];
         super.set(robotQuadrant, storageWhenNull);
         fillValueArray();
         return element;
      }

      return element;
   }

   public V remove(RobotQuadrant robotQuadrant)
   {
      // remove
      if (containsQuadrant(robotQuadrant))
      {
         elementStorageForWhenNull[robotQuadrant.ordinal()] = get(robotQuadrant);
         V remove = super.remove(robotQuadrant);
         fillValueArray();
         return remove;
      }
      // do nothing
      else
      {
         return get(robotQuadrant);
      }
   }

   private void fillValueArray()
   {
      if (size() == 4)
         return;

      for (int i = 0, j = 0; i < elementStorageForWhenNull.length; i++)
      {
         if (containsQuadrant(RobotQuadrant.values[i]))
         {
            valueArrays[size()][j++] = elementStorageForWhenNull[i];
         }
      }
   }

   public V[] keys()
   {
      return valueArrays[size()];
   }

   @Override
   public void clear()
   {
      for (RobotQuadrant robotQuadrant : quadrants())
      {
         elementStorageForWhenNull[robotQuadrant.ordinal()] = get(robotQuadrant);
      }

      super.clear();
   }

   public boolean containsQuadrant(RobotQuadrant robotQuadrant)
   {
      return this.containsKey(robotQuadrant);
   }
}
