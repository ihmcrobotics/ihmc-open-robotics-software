package us.ihmc.tools.reflect;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;

import us.ihmc.tools.containers.ContainerTools;

/**
 * @author Twan
 *
 */
public class RecursiveObjectComparer
{
   private Object topLevelObject1;
   private Object topLevelObject2;
   private final int maxDepth;
   private final int maxSize;

   private final ArrayList<Field> differingFields = new ArrayList<Field>();
   private final ArrayList<Object[]> secondLowestLevelDifferenceList = new ArrayList<Object[]>();
   private final LinkedHashMap<Object, ArrayList<Object>> secondLowestLevelDifferenceMap = new LinkedHashMap<Object, ArrayList<Object>>();
   private final ArrayList<String> secondLowestLevelDifferenceLocations = new ArrayList<String>();
   private final ArrayList<String[]> lowestLevelDifferencesList = new ArrayList<String[]>();

   private LinkedHashMap<Object, ArrayList<Object>> previouslyProcessedDifferentObjectPairs = new LinkedHashMap<Object, ArrayList<Object>>();
   private LinkedHashMap<Object, ArrayList<Object>> previouslyProcessedEqualObjectPairs = new LinkedHashMap<Object, ArrayList<Object>>();
   private LinkedHashMap<Object, ArrayList<Object>> objectStack = new LinkedHashMap<Object, ArrayList<Object>>();
   private long primitivesProcessed = 0;
   private long fieldsProcessed = 0;
   private final StringBuffer differenceLocationBuffer = new StringBuffer();


   private LinkedHashMap<Object, ArrayList<Object>> objectPairsToIgnore = new LinkedHashMap<Object, ArrayList<Object>>();
   private LinkedHashSet<Field> fieldsToIgnore = new LinkedHashSet<Field>();
   private LinkedHashSet<Class<?>> classesToIgnore = new LinkedHashSet<Class<?>>();
   private StringFieldMatcher stringFieldMatchersToIgnore = new StringFieldMatcher();


   private final boolean isTopLevel;

   /**
    * top level constructor
    */
   public RecursiveObjectComparer(int maxDepth, int maxSize)
   {
      this.maxDepth = maxDepth;
      this.maxSize = maxSize;
      this.isTopLevel = true;
   }

   /**
    * child level constructor (for comparing keys)
    */
   private RecursiveObjectComparer(RecursiveObjectComparer parentComparer, int maxDepth, int maxSize)
   {
      this.maxDepth = maxDepth;
      this.maxSize = maxSize;
      this.isTopLevel = false;

      // link to parent comparer
      this.classesToIgnore = parentComparer.classesToIgnore;    // will not be changed by the child comparer
      this.fieldsToIgnore = parentComparer.fieldsToIgnore;    // will not be changed by the child comparer
      this.stringFieldMatchersToIgnore = parentComparer.stringFieldMatchersToIgnore;
      this.previouslyProcessedEqualObjectPairs = parentComparer.previouslyProcessedEqualObjectPairs;    // stuff will be added by this comparer, but that's to parentComparers benefit
      this.previouslyProcessedDifferentObjectPairs = parentComparer.previouslyProcessedDifferentObjectPairs;    // stuff will be added by this comparer, but that's to parentComparers benefit
      this.objectStack = parentComparer.objectStack;    // have to add this, otherwise loops won't be resolved correctly.
   }

   public boolean compare(Object object1, Object object2) throws IllegalArgumentException, IllegalAccessException
   {
      this.topLevelObject1 = object1;
      this.topLevelObject2 = object2;
      boolean objectsEqual = recursivelyCompareObjects(topLevelObject1, topLevelObject2, maxDepth, null);
      doPostCompareSanityChecks();
   
//      if (!objectsEqual && this.differingFields.isEmpty())
//      {
//         throw new RuntimeException("(ret == false) && this.differingFields.isEmpty()");
//      }
//      if (objectsEqual && !differingFields.isEmpty())
//      {
//         throw new RuntimeException("(ret == true) && !differingFields.isEmpty()");
//      }
      
      return objectsEqual;
   }

   public void clear()
   {
      differingFields.clear();
      secondLowestLevelDifferenceList.clear();
      secondLowestLevelDifferenceMap.clear();
      secondLowestLevelDifferenceLocations.clear();
      lowestLevelDifferencesList.clear();

      previouslyProcessedDifferentObjectPairs.clear();
      previouslyProcessedEqualObjectPairs.clear();
      objectStack.clear();
      primitivesProcessed = 0;
      fieldsProcessed = 0;
      differenceLocationBuffer.setLength(0);
   }

   public boolean isTopLevel()
   {
      return isTopLevel;
   }

   public long getPrimitivesProcessed()
   {
      return primitivesProcessed;
   }

   public long getFieldsProcessed()
   {
      return fieldsProcessed;
   }

   public ArrayList<Field> getDifferingFields()
   {
      return differingFields;
   }

   public ArrayList<Object[]> getLowestLevelObjectDifferenceList()
   {
      return secondLowestLevelDifferenceList;
   }

   public HashMap<Object, ArrayList<Object>> getLowestLevelObjectDifferenceMap()
   {
      return secondLowestLevelDifferenceMap;
   }

   public ArrayList<String> getLowestLevelObjectDifferenceLocations()
   {
      return secondLowestLevelDifferenceLocations;
   }

   public void addFieldToIgnore(Field fieldToIgnore)
   {
      this.fieldsToIgnore.add(fieldToIgnore);
   }

   public void addFieldsToIgnore(Collection<Field> fieldsToIgnore)
   {
      this.fieldsToIgnore.addAll(fieldsToIgnore);
   }

   public void addStringFieldsToIgnore(StringFieldMatcher stringFieldMatcherToIgnore)
   {
      if (stringFieldMatcherToIgnore != null)
      {
         this.stringFieldMatchersToIgnore.combine(stringFieldMatcherToIgnore);
      }
   }
   
   public void addClassToIgnore(Class<?> classToIgnore)
   {
      this.classesToIgnore.add(classToIgnore);
   }

   public void addClassesToIgnore(Collection<? extends Class<?>> classesToIgnore)
   {
      this.classesToIgnore.addAll(classesToIgnore);
   }

   public void addObjectPairsToIgnore(Map<Object, ArrayList<Object>> additionalObjectPairsToIgnore)
   {
      for (Object key : additionalObjectPairsToIgnore.keySet())
      {
         if (this.objectPairsToIgnore.containsKey(key))
         {
            this.objectPairsToIgnore.get(key).addAll(additionalObjectPairsToIgnore.get(key));
         }
         else
         {
            this.objectPairsToIgnore.put(key, new ArrayList<Object>(additionalObjectPairsToIgnore.get(key)));
         }
      }
   }

   private void doPostCompareSanityChecks()
   {
      // everything is processed at this point, so the object stack should be empty
      if (isTopLevel())
      {
         if (!objectStack.isEmpty())
         {
            throw new RuntimeException("object stack should be empty. It contains\n: " + objectStack);
         }
      }

      // size checks
      int size = differingFields.size();
      if (secondLowestLevelDifferenceList.size() != size)
      {
         throw new RuntimeException("Sizes don't match");
      }

      if (secondLowestLevelDifferenceLocations.size() != size)
      {
         throw new RuntimeException("Sizes don't match");
      }

      if (lowestLevelDifferencesList.size() != size)
      {
         throw new RuntimeException("Sizes don't match");
      }
   }

   private boolean recursivelyCompareObjects(Object object1, Object object2, int depthToGo, Field callingField)
           throws IllegalArgumentException, IllegalAccessException
   {
      boolean ret;
      if (hasMaximumNumberOfDifferencesBeenReached())
         ret = true;
      else if (areObjectsIdenticalOrBothNull(object1, object2))
         ret = true;
      else if (isExactlyOneObjectNull(object1, object2))
      {
         storeExactlyOneNullDifference(object1, object2, callingField);
         ret = false;
      }
      // at this point, the objects cannot be null, so we can get their types
      else if (areObjectsOfDifferentClasses(object1, object2))
      {
         storeDifferentClassesDifference(object1, object2, callingField);
         ret = false;
      }
      else
      {
         Class<?> objectType = object1.getClass();    // doesn't matter which one we use, since they're the same at this point

         if (isClassAnException(objectType))
            ret = true;
         else if (isObjectPairAnException(object1, object2))
            ret = true;
         else if (alreadyKnowObjectsAreDifferent(object1, object2))
         {
            ret = false;
         }
         else if (alreadyKnowObjectsAreEqual(object1, object2))
            ret = true;
         else if (areObjectsAlreadyBeingCompared(object1, object2))
         {
            // resolve loops
            ret = true;
         }
         else
         {
            addObjectPairToObjectStack(object1, object2);

            // After this point, the object pair that's just been put on the objectStack must be taken off it before we return

            // put the root class name at the start of the location strings
            if (callingField == null)
            {
               differenceLocationBuffer.append(objectType.getName());
            }

            if (objectType.isArray())
            {
               // handle arrays
               ret = handleArrays(object1, object2, depthToGo, callingField);
               doBookKeeping(object1, object2, ret);
            }
            else if (Map.class.isAssignableFrom(objectType))
            {
               // handle maps
               ret = handleMaps(object1, object2, depthToGo, callingField);
               doBookKeeping(object1, object2, ret);
            }
            else
            {
               // handle other kinds of objects
               ret = compareFields(object1, object2, depthToGo);
            }
         }
      }

      return ret;
   }

   private void doBookKeeping(Object object1, Object object2, boolean equal)
   {
      if (equal)
      {
         registerEqualObjects(object1, object2);
      }
      else
      {
         registerDifferentObjects(object1, object2);
      }

      removeFromObjectStack(object1, object2);
   }

   private boolean compareFields(Object object1, Object object2, int depthToGo)
   {
      if (depthToGo > 0)
      {
         try
         {
            boolean result = tryToCompareFields(object1, object2, depthToGo);
            return result;
         }
         catch (IllegalAccessException e)
         {
            e.printStackTrace();
            removeFromObjectStack(object1, object2);

            return false;
         }
      }
      else
      {
         removeFromObjectStack(object1, object2);

         return true;    // don't know, actually...
      }
   }

   private boolean tryToCompareFields(Object object1, Object object2, int depthToGo) throws IllegalAccessException
   {
      ArrayList<Field> allFields = ReflectionTools.getAllFields(object1.getClass());

      boolean equal = true;
      for (Field field : allFields)
      {
         if (fieldsToIgnore.contains(field))
         {
            continue;
         }
         
         if (stringFieldMatchersToIgnore != null)
         {
            if (stringFieldMatchersToIgnore.matches(object1)) 
            {
               continue;
            }
         }

         int oldLocationBufferLength = differenceLocationBuffer.length();
         differenceLocationBuffer.append("." + getFieldString(field));

         field.setAccessible(true);
         Class<?> fieldType = field.getType();
         boolean isPrimitive = fieldType.isPrimitive();

         if (isPrimitive)
         {
            if (!ReflectionTools.isPrimitiveFieldContentTheSame(object1, object2, field))
            {
               storePrimitiveDifference(object1, object2, field);
               equal = false;
            }

            primitivesProcessed++;
         }
         else
         {
            Object fieldContent1 = field.get(object1);
            Object fieldContent2 = field.get(object2);

            boolean fieldEqual = recursivelyCompareObjects(fieldContent1, fieldContent2, depthToGo - 1, field);
            if (!fieldEqual)
            {
               equal = false;    // design choice: don't return here: keep going to find more differences
            }
         }

         differenceLocationBuffer.setLength(oldLocationBufferLength);
         fieldsProcessed++;
      }

      doBookKeeping(object1, object2, equal);

      return equal;
   }


   private void addObjectPairToObjectStack(Object object1, Object object2)
   {
      ArrayList<Object> objectStackEntry = objectStack.get(object1);
      if (objectStackEntry == null)
      {
         objectStackEntry = new ArrayList<Object>();
         objectStack.put(object1, objectStackEntry);
      }

      objectStackEntry.add(object2);
   }

   private boolean areObjectsAlreadyBeingCompared(Object object1, Object object2)
   {
      ArrayList<Object> objectStackEntry = objectStack.get(object1);
      if (objectStackEntry == null)
         return false;

      boolean objectsAlreadyBeingCompared = objectStackEntry.contains(object2);

      return objectsAlreadyBeingCompared;
   }


   private boolean alreadyKnowObjectsAreEqual(Object object1, Object object2)
   {
      boolean alreadyKnowObjectsAreEqual = (previouslyProcessedEqualObjectPairs.get(object1) != null)
                                           && previouslyProcessedEqualObjectPairs.get(object1).contains(object2);

      return alreadyKnowObjectsAreEqual;
   }

   private boolean alreadyKnowObjectsAreDifferent(Object object1, Object object2)
   {
      boolean alreadyKnowObjectsAreDifferent = (previouslyProcessedDifferentObjectPairs.get(object1) != null)
                                               && previouslyProcessedDifferentObjectPairs.get(object1).contains(object2);

      return alreadyKnowObjectsAreDifferent;
   }

   private boolean isObjectPairAnException(Object object1, Object object2)
   {
      ArrayList<Object> exceptionEntry = objectPairsToIgnore.get(object1);

      return ((exceptionEntry != null) && exceptionEntry.contains(object2));
   }

   private boolean isClassAnException(Class<?> objectType)
   {
      // check if the class is an exception
      for (Class<?> exceptionClass : classesToIgnore)
      {
         if (exceptionClass.isAssignableFrom(objectType))
         {
            return true;
         }
      }

      return false;
   }

   private static boolean areObjectsOfDifferentClasses(Object object1, Object object2)
   {
      Class<?> object1Type = object1.getClass();
      Class<?> object2Type = object2.getClass();

      return (object1Type != object2Type);
   }

   private boolean hasMaximumNumberOfDifferencesBeenReached()
   {
      return (differingFields.size() >= maxSize);
   }

   private static boolean areObjectsIdenticalOrBothNull(Object object1, Object object2)
   {
      return (object1 == object2);
   }

   private static boolean isExactlyOneObjectNull(Object object1, Object object2)
   {
      boolean object1Null = object1 == null;
      boolean object2Null = object2 == null;

      if ((object1Null &&!object2Null) || (!object1Null && object2Null))
      {
         return true;
      }

      return false;
   }

   private void registerEqualObjects(Object object1, Object object2)
   {
      ArrayList<Object> entry = previouslyProcessedEqualObjectPairs.get(object1);
      if (entry == null)
      {
         ArrayList<Object> arrayList = new ArrayList<Object>();
         arrayList.add(object2);
         previouslyProcessedEqualObjectPairs.put(object1, arrayList);
      }
      else
      {
         entry.add(object2);
      }
   }

   private void registerDifferentObjects(Object object1, Object object2)
   {
      ArrayList<Object> entry = previouslyProcessedDifferentObjectPairs.get(object1);
      if (entry == null)
      {
         ArrayList<Object> arrayList = new ArrayList<Object>();
         arrayList.add(object2);
         previouslyProcessedDifferentObjectPairs.put(object1, arrayList);
      }
      else
      {
         entry.add(object2);
      }
   }

   private void removeFromObjectStack(Object object1, Object object2)
   {
      ArrayList<Object> objectStackEntry = objectStack.get(object1);
      if (objectStackEntry != null)
      {
         ContainerTools.removeByReference(objectStackEntry, object2);

         if (objectStackEntry.isEmpty())
         {
            objectStack.remove(object1);
         }
      }
      else
      {
         throw new RuntimeException("Object not found!");
      }
   }

   private void storePrimitiveDifference(Object object1, Object object2, Field callingField) throws IllegalArgumentException, IllegalAccessException
   {
      log(object1, object2, callingField);

      String[] primitiveDifference = getPrimitiveDifference(object1, object2, callingField);
      lowestLevelDifferencesList.add(primitiveDifference);
   }

   private void storePrimitiveDifferenceForArrays(Object array1, Object array2, Field callingField, int arrayIndex)
           throws IllegalArgumentException, IllegalAccessException
   {
      log(array1, array2, callingField);

      String[] primitiveDifference = getPrimitiveDifferenceForArrays(array1, array2, arrayIndex);
      lowestLevelDifferencesList.add(primitiveDifference);
   }

   private void storeArrayLengthDifference(Object array1, Object array2, Field callingField)
   {
      log(array1, array2, callingField);
      
      String[] differenceString = getLengthDifferenceForArrays(array1, array2);
      lowestLevelDifferencesList.add(differenceString);
   }

   private void storeExactlyOneNullDifference(Object object1, Object object2, Field callingField)
   {
      log(object1, object2, callingField);

      String[] primitiveDifference = new String[2];
      primitiveDifference[0] = object1 == null ? "null" : "not null";
      primitiveDifference[1] = object2 == null ? "null" : "not null";
      lowestLevelDifferencesList.add(primitiveDifference);
   }

   private void storeDifferentClassesDifference(Object object1, Object object2, Field callingField)
   {
      log(object1, object2, callingField);

      String[] primitiveDifference = new String[2];
      primitiveDifference[0] = object1.getClass().toString();
      primitiveDifference[1] = object2.getClass().toString();
      lowestLevelDifferencesList.add(primitiveDifference);
   }

   private void log(Object object1, Object object2, Field callingField)
   {
      differingFields.add(callingField);

      Object[] objectDifference = {object1, object2};
      secondLowestLevelDifferenceList.add(objectDifference);

      secondLowestLevelDifferenceLocations.add(differenceLocationBuffer.toString());

      ArrayList<Object> lowestLevelObjectDifferenceMapEntry = secondLowestLevelDifferenceMap.get(object1);
      if (lowestLevelObjectDifferenceMapEntry == null)
      {
         ArrayList<Object> arrayList = new ArrayList<Object>();
         arrayList.add(object2);
         secondLowestLevelDifferenceMap.put(object1, arrayList);
      }
      else
      {
         lowestLevelObjectDifferenceMapEntry.add(object2);
      }
   }

   private static String[] getPrimitiveDifference(Object object1, Object object2, Field callingField) throws IllegalAccessException
   {
      String[] primitiveDifference = new String[2];
      primitiveDifference[0] = ReflectionTools.getStringRepresentationOfFieldContent(object1, callingField);
      primitiveDifference[1] = ReflectionTools.getStringRepresentationOfFieldContent(object2, callingField);

      return primitiveDifference;
   }

   private static String[] getPrimitiveDifferenceForArrays(Object array1, Object array2, int arrayIndex)
   {
      String[] primitiveDifference = new String[2];
      primitiveDifference[0] = ReflectionTools.getStringRepresentationOfArrayEntry(array1, arrayIndex);
      primitiveDifference[1] = ReflectionTools.getStringRepresentationOfArrayEntry(array2, arrayIndex);

      return primitiveDifference;
   }

   private String[] getLengthDifferenceForArrays(Object array1, Object array2)
   {
      String[] differenceString = new String[2];
      differenceString[0] = "length = " + Array.getLength(array1);
      differenceString[1] = "length = " + Array.getLength(array2);
      return differenceString;
   }

   private String getFieldString(Field field)
   {
      String fieldString = field.toString();
      int dotIndex = fieldString.lastIndexOf(".");

      return fieldString.substring(dotIndex + 1);
   }

   private boolean handleArrays(Object array1, Object array2, int depthToGo, Field callingField) throws IllegalArgumentException, IllegalAccessException
   {
      int arrayLength = Array.getLength(array1);
      if (arrayLength != Array.getLength(array2))
      {
         storeArrayLengthDifference(array1, array2, callingField);
         return false;
      }
      else
      {
         boolean isPrimitiveArray = array1.getClass().getComponentType().isPrimitive();
         boolean equal = true;
         for (int i = 0; i < arrayLength; i++)
         {
            int oldLocationBufferLength = differenceLocationBuffer.length();
            differenceLocationBuffer.append("[" + i + "]");
            boolean entryEqual;
            if (isPrimitiveArray)
            {
               entryEqual = ReflectionTools.isPrimitiveArrayEntryTheSame(array1, array2, i);

               if (!entryEqual)
               {
                  storePrimitiveDifferenceForArrays(array1, array2, callingField, i);
               }
            }
            else
            {
               entryEqual = recursivelyCompareObjects(Array.get(array1, i), Array.get(array2, i), depthToGo - 1, callingField);
            }

            if (!entryEqual)
            {
               equal = false;

               // design choice: don't return here; keep going to find more differences
            }

            differenceLocationBuffer.setLength(oldLocationBufferLength);
         }

         return equal;
      }
   }

   private boolean handleMaps(Object map1, Object map2, int depthToGo, Field callingField) throws IllegalArgumentException, IllegalAccessException
   {
      Map<?, ?> map1Cast = (Map<?, ?>) map1;
      Map<?, ?> map2Cast = (Map<?, ?>) map2;

      if (map1Cast.size() != map2Cast.size())
      {
         return false;
      }

      Object[] keySet1 = map1Cast.keySet().toArray();
      Object[] keySet2 = map2Cast.keySet().toArray();

      boolean equal = true;
      key1Loop:
      for (int i = 0; i < keySet1.length; i++)
      {
         Object key1 = keySet1[i];
         boolean keyFound = false;
         for (int j = 0; j < keySet2.length; j++)
         {
            Object key2 = keySet2[j];
            boolean keyEqual = compareKeys(key1, key2, Integer.MAX_VALUE);
            if (keyEqual)
            {
               // correct key found; now compare the corresponding values
               keyFound = true;
               Object value1 = map1Cast.get(key1);
               Object value2 = map2Cast.get(key2);
               boolean valueEqual = recursivelyCompareObjects(value1, value2, depthToGo - 1, callingField);
               if (!valueEqual)
               {
                  equal = false;
               }

               continue key1Loop;    // design choice: don't return here: keep going to find more differences
            }
         }

         // correct key not found
         if (!keyFound && isTopLevel())
         {
//          System.err.println("Could not find map2's counterpart of map1's key " + key1);
            equal = false;
         }
      }

      return equal;
   }

   private boolean compareKeys(Object key1, Object key2, int depthToGo) throws IllegalArgumentException, IllegalAccessException
   {
      RecursiveObjectComparer keyComparer = new RecursiveObjectComparer(this, depthToGo - 1, Integer.MAX_VALUE);
      boolean equal = keyComparer.compare(key1, key2);

      return equal;
   }

   @Override
   public String toString()
   {
      StringBuffer buf = new StringBuffer();

      int maxDifferingFields = 5000;
      int nDifferingFields = differingFields.size();
      int fieldsToBePrinted = Math.min(nDifferingFields, maxDifferingFields);
      for (int i = 0; i < fieldsToBePrinted; i++)
      {
         Field field = differingFields.get(i);
         String location = secondLowestLevelDifferenceLocations.get(i);

         buf.append("Field: " + field + "\n");
         buf.append("Location: " + location + "\n");

         String[] primitiveDifference = lowestLevelDifferencesList.get(i);    // should be synchronized with differingFields (not very nice, but it's late)
         Object[] secondLowestLevelDifference = secondLowestLevelDifferenceList.get(i);
         for (int j = 0; j < primitiveDifference.length; j++)
         {
            buf.append("   Primitive " + (j) + ": " + primitiveDifference[j]);
            
            Object containingObject = secondLowestLevelDifference[j];
            buf.append("   Containing object " + (j) + ": " + containingObject);
            if (containingObject != null)
            {
               buf.append("   Containing object class " + (j) + ": " + containingObject.getClass());               
            }
            buf.append("\n");
         }
      }

      if (fieldsToBePrinted < nDifferingFields)
      {
         buf.append("Output truncated. First " + maxDifferingFields + " differing fields shown.\n");
      }

      buf.append("\n");
      buf.append("Differing field types:\n");
      LinkedHashSet<Field> differingFieldsSet = new LinkedHashSet<Field>(differingFields);
      for (Field field : differingFieldsSet)
      {
         if (field != null)
         {
            buf.append(field.toGenericString());
         }
         else
         {
            buf.append("null");
         }
         buf.append("\n");
      }

      buf.append("\n");
      buf.append("Total number of differing fields: " + nDifferingFields + "\n");

      return buf.toString();
   }
}
