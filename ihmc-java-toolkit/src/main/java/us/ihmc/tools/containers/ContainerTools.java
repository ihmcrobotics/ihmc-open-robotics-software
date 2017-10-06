package us.ihmc.tools.containers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.List;
import java.util.TreeSet;

/**
 * Class contains static functions that are useful when dealing with container classes such as ArrayList, EnumMap, HashMap, etc.
 *
 */
public class ContainerTools
{
   private ContainerTools()
   {
      // empty;
   }

   /**
    * Creates a new EnumMap. This method is a shortcut to make construction less verbose.
    * @param <K> key type
    * @param <V> value type
    * @param keyType runtime key type
    * @return a new EnumMap
    */
   public static <K extends Enum<K>, V> EnumMap<K, V> createEnumMap(Class<K> keyType)
   {
      return new EnumMap<K, V>(keyType);
   }

   /**
    * Finds the n largest elements of the given collection, according to the specified comparator.
    * @param <E> collection type
    * @param collection the collection from which the largest elements need to be found
    * @param comparator compares the elements of the collection.
    * @param n maximum number of elements contained in the returned set
    * @return
    */
   public static <E> TreeSet<E> findLargestElements(Collection<? extends E> collection, Comparator<? super E> comparator, int n)
   {
      TreeSet<E> ret = new TreeSet<E>(comparator);
      for (E element : collection)
      {
         if (ret.size() < n)
         {
            ret.add(element);
         }
         else
         {
            E currentLowest = ret.first();
            if (ret.comparator().compare(element, currentLowest) > 0)
            {
               ret.remove(currentLowest);
               ret.add(element);
            }
         }
      }

      return ret;
   }

   /**
    * Takes all the elements in an EnumMap of EnumMaps and puts them in an ArrayList
    */
   public static <K1 extends Enum<K1>, K2 extends Enum<K2>, V> ArrayList<V> flatten(EnumMap<K1, EnumMap<K2, V>> mapOfMaps)
   {
      ArrayList<V> ret = new ArrayList<V>();
      for (EnumMap<K2, V> map : mapOfMaps.values())
      {
         for (V value : map.values())
         {
            ret.add(value);
         }
      }

      return ret;
   }

   /**
    * Takes all the elements in an EnumMap of EnumMaps and puts them into an array of arrays
    */
   public static <K1 extends Enum<K1>, K2 extends Enum<K2>> double[][] toArrayOfArrays(EnumMap<K1, EnumMap<K2, Double>> mapOfMaps)
   {
      int size1 = mapOfMaps.keySet().size();
      double[][] ret = new double[size1][];
      for (K1 key1 : mapOfMaps.keySet())
      {
         EnumMap<K2, Double> map = mapOfMaps.get(key1);
         int size2 = map.keySet().size();
         ret[key1.ordinal()] = new double[size2];

         for (K2 key2 : map.keySet())
         {
            ret[key1.ordinal()][key2.ordinal()] = mapOfMaps.get(key1).get(key2);
         }
      }

      return ret;
   }

   public static <T extends Comparable<? super T>> List<T> asSortedList(Collection<T> c)
   {
      List<T> list = new ArrayList<T>(c);
      java.util.Collections.sort(list);

      return list;
   }

   /**
    * Removes an object from a list *by reference*, instead of using the equals method
    * @param list
    * @param o
    * @return
    */
   public static <T> boolean removeByReference(List<T> list, T o)
   {
      if (o == null)
      {
         list.remove(o);
      }
      else
      {
         for (int index = 0; index < list.size(); index++)
         {
            if (o == list.get(index))
            {
               list.remove(index);
               return true;
            }
         }
      }
      return false;
   }
}
