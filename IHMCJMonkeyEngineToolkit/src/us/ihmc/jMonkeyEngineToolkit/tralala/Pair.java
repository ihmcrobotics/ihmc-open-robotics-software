package us.ihmc.jMonkeyEngineToolkit.tralala;

/** 
 * Encapsulation of 2 objects, the key and its value.
 */
public class Pair<Key, Value>
{
   private Key key;
   private Value value;

   public Pair(Key key, Value value)
   {
      this.key = key;
      this.value = value;
   }

   public Pair(Key key)
   {
      this.key = key;
   }

   public Key getKey()
   {
      return key;
   }

   public Value getValue()
   {
      return value;
   }

   public void setKey(Key key)
   {
      this.key = key;
   }

   public void setValue(Value value)
   {
      this.value = value;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == null) return false;
      if (getClass() != obj.getClass()) return false;
      final Pair<Key, Value> other = (Pair<Key, Value>) obj;
      if (this.key != other.key && (this.key == null || !this.key.equals(other.key))) return false;
      return true;
   }

   @Override
   public int hashCode()
   {
      return key.hashCode();
   }

   @Override
   public String toString()
   {
      return "{"+ key + "," + value + '}';
   }
}
