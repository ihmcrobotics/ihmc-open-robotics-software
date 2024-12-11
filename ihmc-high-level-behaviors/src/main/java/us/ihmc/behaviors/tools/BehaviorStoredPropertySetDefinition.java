package us.ihmc.behaviors.tools;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.TextNode;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.map.hash.TObjectByteHashMap;
import gnu.trove.map.hash.TObjectDoubleHashMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.communication.crdt.CRDTBidirectionalStoredPropertySet;
import us.ihmc.communication.crdt.RequestConfirmFreezable;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;
import us.ihmc.tools.property.StoredPropertySetReadOnly;

public class BehaviorStoredPropertySetDefinition extends CRDTBidirectionalStoredPropertySet
{
   private final String name;
   private final TObjectDoubleHashMap<DoubleStoredPropertyKey> defaultDoubleValues = new TObjectDoubleHashMap<>();
   private final TObjectIntHashMap<IntegerStoredPropertyKey> defaultIntegerValues = new TObjectIntHashMap<>();
   private final TObjectByteHashMap<BooleanStoredPropertyKey> defaultBooleanValues = new TObjectByteHashMap<>();
   private final TDoubleArrayList onDiskDoubleValues = new TDoubleArrayList();
   private final TIntArrayList onDiskIntegerValues = new TIntArrayList();
   private final TByteArrayList onDiskBooleanValues = new TByteArrayList();

   public BehaviorStoredPropertySetDefinition(RequestConfirmFreezable requestConfirmFreezable,
                                              String name,
                                              StoredPropertySetBasics storedPropertySet)
   {
      super(requestConfirmFreezable, storedPropertySet);

      this.name = name;

      storedPropertySet.getKeyList().keys().forEach(key ->
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            defaultDoubleValues.put(doubleKey, storedPropertySet.get(doubleKey));
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            defaultIntegerValues.put(integerKey, storedPropertySet.get(integerKey));
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            defaultBooleanValues.put(booleanKey, booleanToByte(storedPropertySet.get(booleanKey)));
         }
      });

      setOnDiskFields();
   }

   public void toJSON(ObjectNode jsonNode)
   {
      StoredPropertySetReadOnly storedPropertySet = getValueReadOnly();
      ObjectNode objectNode = jsonNode.putObject(name);

      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey
             && !(defaultDoubleValues.get(doubleKey) == storedPropertySet.get(doubleKey)))
         {
            objectNode.put(doubleKey.getTitleCasedName(), "%s -> %s".formatted(defaultDoubleValues.get(doubleKey),
                                                                               storedPropertySet.get(doubleKey)));
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey
                  && !(defaultIntegerValues.get(integerKey) == storedPropertySet.get(integerKey)))
         {
            objectNode.put(integerKey.getTitleCasedName(), "%s -> %s".formatted(defaultIntegerValues.get(integerKey),
                                                                                storedPropertySet.get(integerKey)));
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey
                  && !(byteToBoolean(defaultBooleanValues.get(booleanKey)) == storedPropertySet.get(booleanKey)))
         {
            objectNode.put(booleanKey.getTitleCasedName(), "%s -> %s".formatted(byteToBoolean(defaultBooleanValues.get(booleanKey)),
                                                                                storedPropertySet.get(booleanKey)));
         }
      }
   }

   public void fromJSON(JsonNode jsonNode)
   {
      StoredPropertySetBasics storedPropertySet = getValueAndFreeze();
      if (jsonNode.get(name) instanceof ObjectNode objectNode)
      {
         for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
         {
            if (objectNode.get(key.getTitleCasedName()) instanceof TextNode textNode)
            {
               String valueText = textNode.asText().substring(textNode.asText().indexOf("-> ") + 3);
               if (key instanceof DoubleStoredPropertyKey doubleKey)
               {
                  storedPropertySet.set(doubleKey, Double.parseDouble(valueText));
               }
               else if (key instanceof IntegerStoredPropertyKey integerKey)
               {
                  storedPropertySet.set(integerKey, Integer.parseInt(valueText));
               }
               else if (key instanceof BooleanStoredPropertyKey booleanKey)
               {
                  storedPropertySet.set(booleanKey, Boolean.parseBoolean(valueText));
               }
            }
         }
      }
   }

   public void setOnDiskFields()
   {
      onDiskDoubleValues.clear();
      onDiskIntegerValues.clear();
      onDiskBooleanValues.clear();

      StoredPropertySetReadOnly storedPropertySet = getValueReadOnly();
      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            onDiskDoubleValues.add(storedPropertySet.get(doubleKey));
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            onDiskIntegerValues.add(storedPropertySet.get(integerKey));
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            onDiskBooleanValues.add(booleanToByte(storedPropertySet.get(booleanKey)));
         }
      }
   }

   public void undoAllNontopologicalChanges()
   {
      int doubleIndex = 0;
      int integerIndex = 0;
      int booleanIndex = 0;

      StoredPropertySetBasics storedPropertySet = getValueAndFreeze();
      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            storedPropertySet.set(doubleKey, onDiskDoubleValues.get(doubleIndex));
            ++doubleIndex;
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            storedPropertySet.set(integerKey, onDiskIntegerValues.get(integerIndex));
            ++integerIndex;
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            storedPropertySet.set(booleanKey, byteToBoolean(onDiskBooleanValues.get(booleanIndex)));
            ++booleanIndex;
         }
      }
   }

   public boolean isUnchanged()
   {
      boolean unchanged = true;

      int doubleIndex = 0;
      int integerIndex = 0;
      int booleanIndex = 0;

      StoredPropertySetReadOnly storedPropertySet = getValueReadOnly();
      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            unchanged &= storedPropertySet.get(doubleKey) == onDiskDoubleValues.get(doubleIndex);
            ++doubleIndex;
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            unchanged &= storedPropertySet.get(integerKey) == onDiskIntegerValues.get(integerIndex);
            ++integerIndex;
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            unchanged &= storedPropertySet.get(booleanKey) == byteToBoolean(onDiskBooleanValues.get(booleanIndex));
            ++booleanIndex;
         }
      }

      return unchanged;
   }

   private boolean byteToBoolean(byte b)
   {
      return b == (byte) 1;
   }

   private byte booleanToByte(boolean b)
   {
      return b ? (byte) 1 : (byte) 0;
   }
}
