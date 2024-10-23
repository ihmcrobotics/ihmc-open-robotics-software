package us.ihmc.commons.lists;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import java.util.ArrayList;

public class TripleList<T, U, V> extends ArrayList<ImmutableTriple<T, U, V>>
{
   private static final long serialVersionUID = 4769419450370633567L;
  
   public void add(T first, U second, V third)
   {
      add(new ImmutableTriple<T, U, V>(first, second, third));
   }
   
   public T first(int i)
   {
      return get(i).getLeft();
   }
   
   public U second(int i)
   {
      return get(i).getMiddle();
   }

   public V third(int i)
   {
      return get(i).getRight();
   }
}
