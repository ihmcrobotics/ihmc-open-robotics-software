package us.ihmc.robotics.robotSide;

@SuppressWarnings("unchecked")
public class RecyclingQuadrantDependentList<V> extends QuadrantDependentList<V>
{
   private final V[] elementStorageForWhenNull = (V[]) new Object[4];
   private final GenericTypeAdapter<V> genericTypeAdapter; 

   public RecyclingQuadrantDependentList(GenericTypeAdapter<V> genericTypeAdapter)
   {
      super();
      
      this.genericTypeAdapter = genericTypeAdapter;
      
      for (int i = 0; i < 4; i++)
      {
         V newV = genericTypeAdapter.makeANewV();
         
         if (newV == null)
         {
            throw new RuntimeException("New V cannot be null.");
         }
         
         elementStorageForWhenNull[i] = newV;
      }
   }
   
   public interface GenericTypeAdapter<V>
   {
      public V makeANewV();
      
      public void setAV(V newV, V setThisV);
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

   @Override
   public void set(RobotQuadrant robotQuadrant, V element)
   {
      // do nothing
      if (element == get(robotQuadrant))
      {
         return;
      }
      // remove
      if (element == null && containsQuadrant(robotQuadrant))
      {
         super.set(robotQuadrant, element);
         return;
      }
      // add
      else if (element != null && !containsQuadrant(robotQuadrant))
      {
         V storageWhenNull = elementStorageForWhenNull[robotQuadrant.ordinal()];
         genericTypeAdapter.setAV(element, storageWhenNull);
         super.set(robotQuadrant, storageWhenNull);
         return;
      }
      // replace
      else if (element != get(robotQuadrant))
      {
         genericTypeAdapter.setAV(element, get(robotQuadrant));
      }
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
