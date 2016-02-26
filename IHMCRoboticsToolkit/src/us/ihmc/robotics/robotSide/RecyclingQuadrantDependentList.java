package us.ihmc.robotics.robotSide;

import java.lang.reflect.Array;

import us.ihmc.robotics.lists.GenericTypeBuilder;

@SuppressWarnings("unchecked")
public class RecyclingQuadrantDependentList<V>
{
   private final V[] elementStorageForWhenNull;
   private final GenericTypeBuilder<V> builder;   
   private final V[][] valueArrays;
   private final QuadrantDependentList<V> quadrantDependentList;

   public RecyclingQuadrantDependentList(Class<V> clazz)
   {
      quadrantDependentList = new QuadrantDependentList<>();
      builder = GenericTypeBuilder.createBuilderWithEmptyConstructor(clazz);
      
      elementStorageForWhenNull = (V[]) Array.newInstance(clazz, 4);
      for (int i = 0; i < 4; i++)
      {
         V newInstance = builder.newInstance();
         
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
         quadrantDependentList.set(robotQuadrant, element);
         fillValueArray();
         return element;
      }
      // add
      else if (element != null && !containsQuadrant(robotQuadrant))
      {
         V storageWhenNull = elementStorageForWhenNull[robotQuadrant.ordinal()];
         quadrantDependentList.set(robotQuadrant, storageWhenNull);
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
         V remove = quadrantDependentList.remove(robotQuadrant);
         fillValueArray();
         return remove;
      }
      // do nothing
      else
      {
         return get(robotQuadrant);
      }
   }

   public V get(RobotQuadrant key)
   {
      return quadrantDependentList.get(key);
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
   
   public V[] values()
   {
      return valueArrays[size()];
   }
   
   public RobotQuadrant[] quadrants()
   {
      return quadrantDependentList.quadrants();
   }

   public void clear()
   {
      for (RobotQuadrant robotQuadrant : quadrants())
      {
         elementStorageForWhenNull[robotQuadrant.ordinal()] = get(robotQuadrant);
      }

      quadrantDependentList.clear();
   }
   
   public int size()
   {
      return quadrantDependentList.size();
   }
   
   public boolean containsQuadrant(RobotQuadrant robotQuadrant)
   {
      return quadrantDependentList.containsQuadrant(robotQuadrant);
   }
}
