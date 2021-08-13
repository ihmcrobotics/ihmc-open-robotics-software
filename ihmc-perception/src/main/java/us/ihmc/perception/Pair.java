package us.ihmc.perception;

import java.util.AbstractMap;

//from http://stackoverflow.com/questions/1195206/is-there-a-java-equivalent-or-methodology-for-the-typedef-keyword-in-c
public class Pair<T1, T2> extends AbstractMap.SimpleImmutableEntry<T1, T2>
{
   public Pair(T1 key, T2 value)
   {
      super(key, value);
   }
}
