package us.ihmc.tools.lists;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;

public class PairList<T, V> extends ArrayList<ImmutablePair<T, V>>
{
   private static final long serialVersionUID = 4769419450370633567L;
  
   public void add(T first, V second)
   {
      add(new ImmutablePair<T, V>(first, second));
   }
   
   public T first(int i)
   {
      return get(i).getLeft();
   }
   
   public V second(int i)
   {
      return get(i).getRight();
   }
}
