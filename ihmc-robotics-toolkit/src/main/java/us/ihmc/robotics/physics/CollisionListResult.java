package us.ihmc.robotics.physics;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Spliterator;
import java.util.stream.Collector;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

public class CollisionListResult implements Collection<CollisionResult>
{
   private final List<CollisionResult> collisionResults = new ArrayList<>();

   public CollisionListResult()
   {
   }

   @Override
   public void clear()
   {
      collisionResults.clear();
   }

   @Override
   public boolean add(CollisionResult collisionResult)
   {
      return collisionResults.add(collisionResult);
   }

   @Override
   public boolean addAll(Collection<? extends CollisionResult> collisionResults)
   {
      return this.collisionResults.addAll(collisionResults);
   }

   public CollisionResult get(int index)
   {
      return collisionResults.get(index);
   }

   @Override
   public boolean remove(Object object)
   {
      return collisionResults.remove(object);
   }

   @Override
   public boolean removeAll(Collection<?> collection)
   {
      return collisionResults.removeAll(collection);
   }

   @Override
   public boolean retainAll(Collection<?> collection)
   {
      return collisionResults.retainAll(collection);
   }

   @Override
   public boolean contains(Object object)
   {
      return collisionResults.contains(object);
   }

   @Override
   public boolean containsAll(Collection<?> collection)
   {
      return collisionResults.containsAll(collection);
   }

   @Override
   public CollisionResult[] toArray()
   {
      return collisionResults.toArray(new CollisionResult[size()]);
   }

   @Override
   public <T> T[] toArray(T[] array)
   {
      return collisionResults.toArray(array);
   }

   @Override
   public int size()
   {
      return collisionResults.size();
   }

   @Override
   public boolean isEmpty()
   {
      return collisionResults.isEmpty();
   }

   @Override
   public Iterator<CollisionResult> iterator()
   {
      return collisionResults.iterator();
   }

   @Override
   public Spliterator<CollisionResult> spliterator()
   {
      return collisionResults.spliterator();
   }

   @Override
   public int hashCode()
   {
      return collisionResults.hashCode();
   }

   @Override
   public boolean equals(Object object)
   {
      return collisionResults.equals(object);
   }

   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getCollectionString("\n", this, Object::toString);
   }

   public static Collector<CollisionResult, CollisionListResult, CollisionListResult> collector()
   {
      return Collector.of(CollisionListResult::new, CollisionListResult::add, (left, right) ->
      {
         left.addAll(right);
         return left;
      }, Collector.Characteristics.IDENTITY_FINISH);
   }
}
