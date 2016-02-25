package us.ihmc.robotics.robotSide;

import us.ihmc.robotics.lists.GenericTypeBuilder;

@SuppressWarnings("unchecked")
public class RecyclingQuadrantDependentList<V> extends QuadrantDependentList<V>
{
   private final V[] elementStorageForWhenNull = (V[]) new Object[4];
   private final GenericTypeBuilder<V> builder;

   public RecyclingQuadrantDependentList(Class<V> clazz)
   {
      super();
      
      builder = GenericTypeBuilder.createBuilderWithEmptyConstructor(clazz);
      
      for (int i = 0; i < 4; i++)
      {
         V newInstance = builder.newInstance();
         
         elementStorageForWhenNull[i] = newInstance;
      }
   }
   
   /**
    * Ensures quadrant has an element in it and return it.
    * Revived element from temp storage will be dirty.
    * 
    * @param robotQuadrant
    * @return element in quadrant
    */
   public V revive(RobotQuadrant robotQuadrant)
   {
      if (!containsQuadrant(robotQuadrant))
      {
         V storageWhenNull = elementStorageForWhenNull[robotQuadrant.ordinal()];
         super.set(robotQuadrant, storageWhenNull);
         return storageWhenNull;
      }
      else
      {
         return get(robotQuadrant);
      }
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
         return element;
      }
      // add
      else if (element != null && !containsQuadrant(robotQuadrant))
      {
         V storageWhenNull = elementStorageForWhenNull[robotQuadrant.ordinal()];
         super.set(robotQuadrant, storageWhenNull);
         return element;
      }
      
      return element;
   }

   @Override
   public V remove(RobotQuadrant robotQuadrant)
   {
      // remove
      if (containsQuadrant(robotQuadrant))
      {
         elementStorageForWhenNull[robotQuadrant.ordinal()] = get(robotQuadrant);
         return super.remove(robotQuadrant);
      }
      // do nothing
      else
      {
         return get(robotQuadrant);
      }
   }

   @Override
   public V get(RobotQuadrant key)
   {
      return super.get(key);
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
}
