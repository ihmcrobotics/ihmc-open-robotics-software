package us.ihmc.tools.maps;

import sun.misc.SharedSecrets;

//TODO: Implement size, iterator
public class EnumDoubleMap<K extends Enum<K>>
{
   private final double[] vals;

   public EnumDoubleMap(Class<K> keyType)
   {
      K[] universe = SharedSecrets.getJavaLangAccess().getEnumConstantsShared(keyType);
      vals = new double[universe.length];
      
      for(int i = 0; i < vals.length; i++)
      {
         vals[i] = Double.NaN;
      }
   }
   
   public void put(K key, double value)
   {
      vals[key.ordinal()] = value;
   }
   
   public void remove(K key)
   {
      vals[key.ordinal()] = Double.NaN;
   }

   public double get(K key)
   {
      return vals[key.ordinal()];
   }
}
