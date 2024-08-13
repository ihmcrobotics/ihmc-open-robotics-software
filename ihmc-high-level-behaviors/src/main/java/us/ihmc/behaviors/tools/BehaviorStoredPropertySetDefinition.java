package us.ihmc.behaviors.tools;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.BooleanNode;
import com.fasterxml.jackson.databind.node.DoubleNode;
import com.fasterxml.jackson.databind.node.LongNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.communication.crdt.CRDTUnidirectionalStoredPropertySet;
import us.ihmc.communication.crdt.RequestConfirmFreezable;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;
import us.ihmc.tools.property.StoredPropertySetBasics;

public class BehaviorStoredPropertySetDefinition extends CRDTUnidirectionalStoredPropertySet
{
   private final StoredPropertySetBasics storedPropertySet;
   private final TDoubleArrayList onDiskDoubleValues = new TDoubleArrayList();
   private final TIntArrayList onDiskIntegerValues = new TIntArrayList();
   private final TByteArrayList onDiskBooleanValues = new TByteArrayList();

   public BehaviorStoredPropertySetDefinition(ROS2ActorDesignation sideThatCanModify,
                                              RequestConfirmFreezable requestConfirmFreezable,
                                              StoredPropertySetBasics storedPropertySet)
   {
      super(sideThatCanModify, requestConfirmFreezable, storedPropertySet);

      this.storedPropertySet = storedPropertySet;
      setOnDiskFields();
   }

   public void toJSON(ObjectNode jsonNode)
   {
      ObjectNode objectNode = jsonNode.putObject(storedPropertySet.getTitle());

      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            objectNode.put(doubleKey.getTitleCasedName(), storedPropertySet.get(doubleKey));
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            objectNode.put(integerKey.getTitleCasedName(), storedPropertySet.get(integerKey));
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            objectNode.put(booleanKey.getTitleCasedName(), storedPropertySet.get(booleanKey));
         }
      }
   }

   public void fromJSON(JsonNode jsonNode)
   {
      if (jsonNode.get(storedPropertySet.getTitle()) instanceof ObjectNode objectNode)
      {
         for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
         {
            if (key instanceof DoubleStoredPropertyKey doubleKey && objectNode.get(key.getTitleCasedName()) instanceof DoubleNode doubleNode)
            {
               storedPropertySet.set(doubleKey, doubleNode.doubleValue());
            }
            else if (key instanceof IntegerStoredPropertyKey integerKey && objectNode.get(key.getTitleCasedName()) instanceof LongNode longNode)
            {
               storedPropertySet.set(integerKey, longNode.intValue());
            }
            else if (key instanceof BooleanStoredPropertyKey booleanKey && objectNode.get(key.getTitleCasedName()) instanceof BooleanNode booleanNode)
            {
               storedPropertySet.set(booleanKey, booleanNode.booleanValue());
            }
         }
      }
   }

   public void setOnDiskFields()
   {
      onDiskDoubleValues.clear();
      onDiskIntegerValues.clear();
      onDiskBooleanValues.clear();

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
            onDiskBooleanValues.add(storedPropertySet.get(booleanKey) ? (byte) 1 : (byte) 0);
         }
      }
   }

   public void undoAllNontopologicalChanges()
   {
      int doubleIndex = 0;
      int integerIndex = 0;
      int booleanIndex = 0;

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
            storedPropertySet.set(booleanKey, onDiskBooleanValues.get(booleanIndex) == (byte) 1);
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
            unchanged &= storedPropertySet.get(booleanKey) == (onDiskBooleanValues.get(booleanIndex) == (byte) 1);
            ++booleanIndex;
         }
      }

      return unchanged;
   }
}
