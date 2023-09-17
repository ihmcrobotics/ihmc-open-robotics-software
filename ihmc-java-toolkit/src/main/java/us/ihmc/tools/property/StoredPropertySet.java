package us.ihmc.tools.property;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.*;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.commons.thread.Notification;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.*;
import us.ihmc.tools.string.StringTools;

import java.io.InputStream;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

/**
 * Provides a load/saveable property set accessed by strongly typed static keys.
 * <p>
 * The property JSON file is saved to the classpath by file and loaded from the classpath by resource.
 * <p>
 * Some of the benefits of this framework:
 * <li>Keys are created with title cased names available for GUI fields</li>
 * <li>Lightweight and can be used as part of other frameworks</li>
 * <li>JSON file can be placed in higher level projects to override the defaults</li>
 * <p>
 * To create a StoredPropertySet, create a main, us.ihmc.YourStoredPropertySet.java
 *
 * <pre>
 * public static void main(String[] args)
 * {
 *    StoredPropertySet parameters = new StoredPropertySet(keys, YourStoredPropertySet.class);
 *    parameters.generateJavaFiles();
 * }
 * </pre>
 *
 * where the paths are replaced to match your situation. The subsequest paths may be shorter or longer depending on how nested the
 * projects are. Typically, the "directory name to assume present" is the name of the repository. These paths are necessary
 * to allow saving the parameters in version control.
 * <p>
 * Then, create a us.ihmc.YourStoredPropertySetName.json in the resources folder.
 * The name should be the exact same as the *.java class.
 * <p>
 * Properties should only be declared in the JSON if using the generator.
 *
 * <pre>
 * {
 *   "title": "Stored property set name",
 *   "The first boolean property": false,
 *   "The first double property": 0.5,
 *   "The first integer property": 3
 * }
 * </pre>
 *
 * Run the main, which will generate Basics and ReadOnly interfaces and rewrite the main class to extend and implement the correct classes.
 * <p>
 * In order to add descriptions to parameters, you may add it by making the value a JSON array
 * with the 2nd value being the description. You may mix parameters that have descriptions with ones
 * that don't. Descriptions have to be a single long line in JSON, so it is recommended to enable word wrap
 * for JSON files in your editor.
 *
 *  <pre>
 *  {
 *    "title": "Stored property set name",
 *    "The first boolean property": [false, "This is an example test property."],
 *    "The first double property": 0.5,
 *    "The first integer property": [3, "The property before this one doesn't have a description."]
 *  }
 *  </pre>
 *
 */
public class StoredPropertySet implements StoredPropertySetBasics
{
   private final StoredPropertyKeyList keys;
   private final Object[] values;
   private String title;
   private String legacyFileNameINI;
   private String saveFileNameJSON;
   private String currentVersionSuffix;
   private final Class<?> classForLoading;
   private final Class<?> basePropertySetClass;
   private final WorkspaceResourceDirectory workspaceDirectory;
   private final String uncapitalizedClassName;
   private final String capitalizedClassName;
   private WorkspaceResourceFile workspaceLegacyINIFile;
   private WorkspaceResourceFile workspaceJSONFile;

   private final Map<StoredPropertyKey, List<Runnable>> propertyChangedListeners = new HashMap<>();
   private final List<Notification> anyPropertyChangedListeners = new ArrayList<>();

   public StoredPropertySet(StoredPropertyKeyList keys, Class<?> classForLoading)
   {
      this(keys, classForLoading, "");
   }

   public StoredPropertySet(StoredPropertyKeyList keys, Class<?> classForLoading, String versionSuffix)
   {
      this(keys, classForLoading, classForLoading, versionSuffix);
   }

   public StoredPropertySet(StoredPropertyKeyList keys, Class<?> classForLoading, Class<?> basePropertySetClass, String versionSuffix)
   {
      this.keys = keys;
      this.uncapitalizedClassName = StringUtils.uncapitalize(basePropertySetClass.getSimpleName());
      this.capitalizedClassName = basePropertySetClass.getSimpleName();
      this.classForLoading = classForLoading;
      title = classForLoading.getSimpleName();
      this.basePropertySetClass = basePropertySetClass;
      workspaceDirectory = new WorkspaceResourceDirectory(classForLoading);

      updateBackingSaveFile(versionSuffix);
      values = new Object[keys.keys().size()];

      for (StoredPropertyKey<?> key : keys.keys())
      {
         if (key.hasDefaultValue())
         {
            setInternal(key, key.getDefaultValue());
         }
      }
   }

   public void generateJavaFiles()
   {
      StoredPropertySetJavaGenerator generator = new StoredPropertySetJavaGenerator(basePropertySetClass,
                                                                                    classForLoading,
                                                                                    WorkspacePathTools.removePathPartsBeforeProjectFolder(findFileForSaving()));
      if (jsonResourceExists())
      {
         generator.loadFromJSON();
      }
      else // Handle it as it was loaded from INI
      {
         generator.loadFromStoredPropertySet(this);
      }
      generator.generate();

      // Automatically upgrade the stored file to JSON
      if (!jsonResourceExists())
      {
         load();
         save();
      }
   }

   @Override
   public double get(DoubleStoredPropertyKey key)
   {
      Object value = values[key.getIndex()];
      boolean isNull = value == null;
      if (isNull)
         LogTools.warn("Value for key {} is null. Returning Double.NaN.", key.getTitleCasedName());
      return isNull ? Double.NaN : (Double) value;
   }

   @Override
   public int get(IntegerStoredPropertyKey key)
   {
      Object value = values[key.getIndex()];
      boolean isNull = value == null;
      if (isNull)
         LogTools.warn("Value for key {} is null. Returning -1.", key.getTitleCasedName());
      return isNull ? -1 : (Integer) value;
   }

   @Override
   public boolean get(BooleanStoredPropertyKey key)
   {
      Object value = values[key.getIndex()];
      boolean isNull = value == null;
      if (isNull)
         LogTools.warn("Value for key {} is null. Returning false.", key.getTitleCasedName());
      return isNull ? false : (Boolean) value;
   }

   /**
    * @return the value or null
    */
   @Override
   public <T> T get(StoredPropertyKey<T> key)
   {
      Object value = values[key.getIndex()];
      boolean isNull = value == null;
      if (isNull)
         LogTools.warn("Value for key {} is null. Returning null.", key.getTitleCasedName());
      return isNull ? null : (T) value;
   }

   @Override
   public void set(DoubleStoredPropertyKey key, double value)
   {
      setInternal(key, value);
   }

   @Override
   public void set(IntegerStoredPropertyKey key, int value)
   {
      setInternal(key, value);
   }

   @Override
   public void set(BooleanStoredPropertyKey key, boolean value)
   {
      setInternal(key, value);
   }

   @Override
   public <T> void set(StoredPropertyKey<T> key, T value)
   {
      setInternal(key, value);
   }

   @Override
   public <T> StoredProperty<T> getProperty(StoredPropertyKey<T> key)
   {
      return new StoredProperty<>(key, this);
   }

   @Override
   public List<Object> getAll()
   {
      return Arrays.asList(values);
   }

   @Override
   public List<String> getAllAsStrings()
   {
      ArrayList<String> stringValues = new ArrayList<>();
      for (StoredPropertyKey<?> key : keys.keys())
      {
         stringValues.add(serializeValue(get(key)));
      }

      return stringValues;
   }

   @Override
   public void setAll(List<Object> newValues)
   {
      for (int i = 0; i < keys.keys().size(); i++)
      {
         setInternal(keys.keys().get(i), newValues.get(i));
      }
   }

   @Override
   public void setAllFromStrings(List<String> stringValues)
   {
      for (int i = 0; i < keys.keys().size(); i++)
      {
         setInternal(keys.keys().get(i), deserializeString(keys.keys().get(i), stringValues.get(i)));
      }
   }

   @Override
   public void set(StoredPropertySetReadOnly other)
   {
      setAll(other.getAll());
   }

   private void setInternal(StoredPropertyKey key, Object newValue)
   {
      boolean valueChanged;
      if (values[key.getIndex()] == null)
      {
         valueChanged = newValue != null;
      }
      else
      {
         valueChanged = !values[key.getIndex()].equals(newValue);
      }

      if (valueChanged)
      {
         if (!key.getType().equals(newValue.getClass()))
         {
            if (key.getType().equals(Boolean.class) && newValue.getClass().equals(Integer.class))
            {
               newValue = (Integer) newValue != 0;
            }
            else
            {
               throw new RuntimeException("Value of type " + newValue.getClass() + " cannot be set to key type " + key.getType());
            }
         }

         values[key.getIndex()] = newValue;

         if (propertyChangedListeners.get(key) != null)
         {
            for (Runnable propertyChangedListener : propertyChangedListeners.get(key))
            {
               propertyChangedListener.run();
            }
         }
         for (Notification anyPropertyChangedListener : anyPropertyChangedListeners)
         {
            anyPropertyChangedListener.set();
         }
      }
   }

   @Override
   public void addPropertyChangedListener(StoredPropertyKey key, Runnable onPropertyChanged)
   {
      if (propertyChangedListeners.get(key) == null)
      {
         propertyChangedListeners.put(key, new ArrayList<>());
      }

      propertyChangedListeners.get(key).add(onPropertyChanged);
   }

   @Override
   public void removePropertyChangedListener(StoredPropertyKey key, Runnable onPropertyChanged)
   {
      if (propertyChangedListeners.get(key) != null)
      {
         propertyChangedListeners.get(key).remove(onPropertyChanged);
      }
   }

   @Override
   public void addAnyPropertyChangedListener(Notification anyPropertyChangedNotification)
   {
      anyPropertyChangedListeners.add(anyPropertyChangedNotification);
   }

   @Override
   public void removeAnyPropertyChangedListener(Notification anyPropertyChangedNotification)
   {
      anyPropertyChangedListeners.remove(anyPropertyChangedNotification);
   }

   public void updateBackingSaveFile(String versionSuffix)
   {
      updateBackingSaveFileSilently(versionSuffix);
      LogTools.info("Updated backing save file: {}", saveFileNameJSON);
   }

   private void updateBackingSaveFileSilently(String versionSuffix)
   {
      currentVersionSuffix = versionSuffix;
      legacyFileNameINI = uncapitalizedClassName + currentVersionSuffix + ".ini";
      workspaceLegacyINIFile = new WorkspaceResourceFile(workspaceDirectory, legacyFileNameINI);
      saveFileNameJSON = basePropertySetClass.getSimpleName() + currentVersionSuffix + ".json";
      workspaceJSONFile = new WorkspaceResourceFile(workspaceDirectory, saveFileNameJSON);
   }

   @Override
   public void load()
   {
      // Load common properties from non-suffixed version, like bounds, description, etc.
      if (!currentVersionSuffix.isEmpty())
      {
         String backupVersionSuffix = currentVersionSuffix;
         updateBackingSaveFileSilently("");
         if (jsonResourceExists())
            load(true);
         updateBackingSaveFileSilently(backupVersionSuffix);
      }

      load(true);
   }

   @Override
   public void load(String fileName)
   {
      load(fileName, true);
   }

   @Override
   public void load(String fileName, boolean crashIfMissingKeys)
   {
      if (!fileName.startsWith(StringUtils.uncapitalize(workspaceDirectory.getClassForLoading().getSimpleName())))
         throw new RuntimeException("This filename " + fileName +
                                    " breaks the contract of the StoredPropertySet API. The filename should be the class name + suffix.");
      fileName = fileName.replace(".ini", "");
      updateBackingSaveFile(fileName.substring(StringUtils.uncapitalize(workspaceDirectory.getClassForLoading().getSimpleName()).length()));
      load(crashIfMissingKeys);
   }

   public void loadUnsafe()
   {
      load(false);
   }

   private void load(boolean crashIfMissingKeys)
   {
      if (jsonResourceExists())
      {
         LogTools.info("Loading parameters from resource: {}", workspaceJSONFile.getPathForResourceLoadingPathFiltered());
         JSONFileTools.loadFromClasspath(classForLoading, workspaceJSONFile.getPathForResourceLoadingPathFiltered(), node ->
         {
            if (node instanceof ObjectNode objectNode)
            {
               if (objectNode.get("title") instanceof TextNode textNode)
               {
                  title = textNode.asText();
               }

               for (StoredPropertyKey<?> key : keys.keys())
               {
                  JsonNode propertyNode = objectNode.get(key.getTitleCasedName());
                  if (propertyNode == null)
                  {
                     if (!crashIfMissingKeys && key.hasDefaultValue())
                     {
                        setInternal(key, key.getDefaultValue());
                        continue;
                     }

                     throw new RuntimeException(workspaceJSONFile.getClasspathResource() + " does not contain key: " + key.getTitleCasedName());
                  }

                  String stringValue;
                  if (propertyNode instanceof ArrayNode arrayNode)
                  {
                     stringValue = arrayNode.get(0).asText();
                     if (stringValue.equals("null"))
                     {
                        LogTools.warn("{} is being loaded as null. Please set it in {}", key.getCamelCasedName(), saveFileNameJSON);
                     }
                     else
                     {
                        setInternal(key, deserializeString(key, stringValue));
                     }
                     key.setDescription(arrayNode.get(1).textValue());
                  }
                  else if (propertyNode instanceof ObjectNode keyObjectNode)
                  {
                     JsonNode valueNode = keyObjectNode.get("value");
                     JsonNode descriptionNode = keyObjectNode.get("description");
                     if (descriptionNode != null)
                        key.setDescription(descriptionNode.textValue());

                     if (key instanceof DoubleStoredPropertyKey doubleKey)
                     {
                        // Values are not present for common files that just provide bounds, description, etc.
                        if (valueNode != null)
                           setInternal(key, valueNode.doubleValue());
                        JsonNode lowerBound = keyObjectNode.get("lowerBound");
                        if (lowerBound != null)
                           doubleKey.setLowerBound(lowerBound.doubleValue());
                        JsonNode upperBound = keyObjectNode.get("upperBound");
                        if (upperBound != null)
                           doubleKey.setUpperBound(upperBound.doubleValue());

                     }
                     else if (key instanceof IntegerStoredPropertyKey integerKey)
                     {
                        if (valueNode != null)
                           setInternal(key, valueNode.intValue());
                        JsonNode lowerBound = keyObjectNode.get("lowerBound");
                        if (lowerBound != null)
                           integerKey.setLowerBound(lowerBound.intValue());
                        JsonNode upperBound = keyObjectNode.get("upperBound");
                        if (upperBound != null)
                           integerKey.setUpperBound(upperBound.intValue());
                        JsonNode validValues = keyObjectNode.get("validValues");
                        if (validValues instanceof ArrayNode validValuesArray)
                        {
                           int[] validValuesPrimitiveArray = new int[validValuesArray.size()];
                           for (int i = 0; i < validValuesArray.size(); i++)
                           {
                              validValuesPrimitiveArray[i] = validValuesArray.get(i).intValue();
                           }
                           integerKey.setValidValues(validValuesPrimitiveArray);
                        }
                     }
                     else if (key instanceof BooleanStoredPropertyKey booleanKey)
                     {
                        if (valueNode != null)
                           setInternal(key, valueNode.booleanValue());
                     }
                  }
                  else
                  {
                     stringValue = propertyNode.asText();
                     if (stringValue.equals("null"))
                     {
                        LogTools.warn("{} is being loaded as null. Please set it in {}", key.getCamelCasedName(), saveFileNameJSON);
                     }
                     else
                     {
                        setInternal(key, deserializeString(key, stringValue));
                     }
                  }
               }
            }
         });
      }
      else if (iniResourceExists()) // fallback to the old INI format
      {
         ExceptionTools.handle(() ->
         {
            Properties properties = new Properties();
            InputStream streamForLoading = workspaceLegacyINIFile.getClasspathResourceAsStream();

            LogTools.info("Loading parameters from resource: {}/{}", classForLoading.getPackageName().replaceAll("\\.", "/"), legacyFileNameINI);
            properties.load(streamForLoading);

            for (StoredPropertyKey<?> key : keys.keys())
            {
               if (!properties.containsKey(key.getCamelCasedName()))
               {
                  if (!crashIfMissingKeys && key.hasDefaultValue())
                  {
                     setInternal(key, key.getDefaultValue());
                     continue;
                  }

                  throw new RuntimeException(workspaceLegacyINIFile.getClasspathResource() + " does not contain key: " + key.getCamelCasedName());
               }

               String stringValue = (String) properties.get(key.getCamelCasedName());

               if (stringValue.equals("null"))
               {
                  LogTools.warn("{} is being loaded as null. Please set it in {}", key.getCamelCasedName(), legacyFileNameINI);
               }
               else
               {
                  setInternal(key, deserializeString(key, stringValue));
               }
            }
         }, DefaultExceptionHandler.PRINT_STACKTRACE);
      }
      else
      {
         LogTools.warn("Parameter file {} could not be found. Values will be null.", workspaceJSONFile.getPathForResourceLoadingPathFiltered());
      }
   }

   private boolean iniResourceExists()
   {
      return workspaceLegacyINIFile.getClasspathResource() != null;
   }

   private boolean jsonResourceExists()
   {
      return workspaceJSONFile.getClasspathResource() != null;
   }

   public void save()
   {
      Path fileForSaving = findFileForSaving();
      if (workspaceDirectory.isFileAccessAvailable())
      {
         LogTools.info(StringTools.format("Saving parameters to workspace: {}", WorkspacePathTools.removePathPartsBeforeProjectFolder(fileForSaving)));
         FileTools.ensureDirectoryExists(workspaceDirectory.getFilesystemDirectory(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }
      else
      {
         LogTools.info("Saving parameters to working directory: {}", fileForSaving);
      }
      JSONFileTools.save(fileForSaving, jsonRootObjectNode ->
      {
         jsonRootObjectNode.put("title", title);
         for (StoredPropertyKey<?> key : keys.keys())
         {
            boolean isDoubleKeyAndHasExtras = false;
            if (key instanceof DoubleStoredPropertyKey doubleKey)
            {
               isDoubleKeyAndHasExtras |= doubleKey.hasLowerBound();
               isDoubleKeyAndHasExtras |= doubleKey.hasUpperBound();
            }
            boolean isIntegerKeyAndHasExtras = false;
            if (key instanceof IntegerStoredPropertyKey integerKey)
            {
               isIntegerKeyAndHasExtras |= integerKey.hasLowerBound();
               isIntegerKeyAndHasExtras |= integerKey.hasUpperBound();
               isIntegerKeyAndHasExtras |= integerKey.hasSpecifiedValidValues();
            }

            if (isDoubleKeyAndHasExtras || isIntegerKeyAndHasExtras)
            {
               ObjectNode keyObjectNode = jsonRootObjectNode.putObject(key.getTitleCasedName());

               boolean valueIsNull = get(key) == null;
               boolean defaultValueIsNull = key.getDefaultValue() == null;
               if (isDoubleKeyAndHasExtras)
               {
                  DoubleStoredPropertyKey doubleKey = (DoubleStoredPropertyKey) key;
                  keyObjectNode.put("value", valueIsNull ? (defaultValueIsNull ? 0.0 : (double) key.getDefaultValue()) : get(doubleKey));
                  if (doubleKey.hasLowerBound())
                     keyObjectNode.put("lowerBound", doubleKey.getLowerBound());
                  if (doubleKey.hasUpperBound())
                     keyObjectNode.put("upperBound", doubleKey.getUpperBound());
               }
               else if (isIntegerKeyAndHasExtras)
               {
                  IntegerStoredPropertyKey integerKey = (IntegerStoredPropertyKey) key;
                  keyObjectNode.put("value", valueIsNull ? (defaultValueIsNull ? 0 : (int) key.getDefaultValue()) : get(integerKey));
                  if (integerKey.hasLowerBound())
                     keyObjectNode.put("lowerBound", integerKey.getLowerBound());
                  if (integerKey.hasUpperBound())
                     keyObjectNode.put("upperBound", integerKey.getUpperBound());
                  if (integerKey.hasSpecifiedValidValues())
                  {
                     ArrayNode validValuesJSONArray = keyObjectNode.putArray("validValues");
                     for (int validValue : integerKey.getValidValues())
                     {
                        validValuesJSONArray.add(validValue);
                     }
                  }
               }

               if (key.hasDescription())
               {
                  keyObjectNode.put("description", key.getDescription());
               }
            }
            else if (key.hasDescription())
            {
               ArrayNode arrayNode = jsonRootObjectNode.putArray(key.getTitleCasedName());
               boolean valueIsNull = get(key) == null;
               boolean defaultValueIsNull = key.getDefaultValue() == null;
               if (key instanceof BooleanStoredPropertyKey booleanKey)
               {
                  arrayNode.add(valueIsNull ? (defaultValueIsNull ? false : (boolean) key.getDefaultValue()) : get(booleanKey));
               }
               else if (key instanceof DoubleStoredPropertyKey doubleKey)
               {
                  arrayNode.add(valueIsNull ? (defaultValueIsNull ? 0.0 : (double) key.getDefaultValue()) : get(doubleKey));
               }
               else if (key instanceof IntegerStoredPropertyKey integerKey)
               {
                  arrayNode.add(valueIsNull ? (defaultValueIsNull ? 0 : (int) key.getDefaultValue()) : get(integerKey));
               }
               arrayNode.add(key.getDescription());
            }
            else
            {
               boolean valueIsNull = get(key) == null;
               boolean defaultValueIsNull = key.getDefaultValue() == null;
               if (key instanceof BooleanStoredPropertyKey booleanKey)
               {
                  jsonRootObjectNode.put(key.getTitleCasedName(), valueIsNull ? (defaultValueIsNull ? false : (boolean) key.getDefaultValue()) : get(booleanKey));
               }
               else if (key instanceof DoubleStoredPropertyKey doubleKey)
               {
                  jsonRootObjectNode.put(key.getTitleCasedName(), valueIsNull ? (defaultValueIsNull ? 0.0 : (double) key.getDefaultValue()) : get(doubleKey));
               }
               else if (key instanceof IntegerStoredPropertyKey integerKey)
               {
                  jsonRootObjectNode.put(key.getTitleCasedName(), valueIsNull ? (defaultValueIsNull ? 0 : (int) key.getDefaultValue()) : get(integerKey));
               }
            }
         }
      });
      convertLineEndingsToUnix(fileForSaving);

      // Automatically upgrade the stored file to JSON
      if (iniResourceExists() && workspaceLegacyINIFile.isFileAccessAvailable())
      {
         FileTools.deleteQuietly(workspaceLegacyINIFile.getFilesystemFile());
      }
   }

   private String serializeValue(Object object)
   {
      if (object == null)
      {
         return "null";
      }
      else
      {
         return object.toString();
      }
   }

   private <T> T deserializeString(StoredPropertyKey<T> key, String serializedValue)
   {
      if (key.getType().equals(Double.class))
      {
         return (T) Double.valueOf(serializedValue);
      }
      else if (key.getType().equals(Integer.class))
      {
         return (T) Integer.valueOf(serializedValue);
      }
      else if (key.getType().equals(Boolean.class))
      {
         return (T) Boolean.valueOf(serializedValue);
      }
      else
      {
         throw new RuntimeException("Please implement String deserialization for type: " + key.getType());
      }
   }

   private int indexOfCamelCaseName(Object camelCaseName)
   {
      for (StoredPropertyKey<?> key : keys.keys())
      {
         if (key.getCamelCasedName().equals(camelCaseName))
         {
            LogTools.info("Index of camel case name {}: {}", camelCaseName, key.getIndex());
            return key.getIndex();
         }
      }
      return 0;
   }

   private void convertLineEndingsToUnix(Path fileForSaving)
   {
      List<String> lines = FileTools.readAllLines(fileForSaving, DefaultExceptionHandler.PRINT_STACKTRACE);
      PrintWriter printer = FileTools.newPrintWriter(fileForSaving, WriteOption.TRUNCATE, DefaultExceptionHandler.PRINT_STACKTRACE);
      lines.forEach(line -> printer.print(line + "\n"));
      printer.close();
   }

   public static void printInitialSaveFileContents(List<StoredPropertyKey<?>> keys)
   {
      for (StoredPropertyKey<?> parameterKey : keys)
      {
         System.out.println(parameterKey.getCamelCasedName() + "=");
      }
   }

   private Path findFileForSaving()
   {
      return findSaveFileDirectory().resolve(saveFileNameJSON);
   }

   /**
    * Find, for example, ihmc-open-robotics-software/ihmc-footstep-planning/src/main/java/us/ihmc/footstepPlanning/graphSearch/parameters
    * or just save the file in the working directory.
    */
   @Override
   public Path findSaveFileDirectory()
   {
      if (workspaceDirectory.isFileAccessAvailable())
      {
         return workspaceDirectory.getFilesystemDirectory();
      }
      else
      {
         return Paths.get("");
      }
   }

   @Override
   public StoredPropertyKeyListReadOnly getKeyList()
   {
      return keys;
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
         return true;
      else if (!(object instanceof StoredPropertySet))
         return false;
      else
      {
         StoredPropertySet other = (StoredPropertySet) object;

         return Objects.deepEquals(values, other.values);
      }
   }

   /**
    * Sets the properties from a colon-comma string containing all the keys
    * in their natural order. This method does not handle partial subsets or
    * out of order parameters. They are assumed to be of the form:
    * <pre>
    * camelCaseKeyName: value, camelCasedKeyName2: value2, camelCasedKeyName3: value2
    * </pre>
    */
   @Override
   public void setFromColonCommaString(String colonCommaString)
   {
      colonCommaString = colonCommaString.replace(",", "");
      Scanner scanner = new Scanner(colonCommaString);
      for (StoredPropertyKey<?> key : keys.keys())
      {
         if (key instanceof DoubleStoredPropertyKey doubleKey)
         {
            while (!scanner.hasNextDouble())
               scanner.next();
            set(doubleKey, scanner.nextDouble());
         }
         else if (key instanceof IntegerStoredPropertyKey integerKey)
         {
            while (!scanner.hasNextInt())
               scanner.next();
            set(integerKey, scanner.nextInt());
         }
         else if (key instanceof BooleanStoredPropertyKey booleanKey)
         {
            while (!scanner.hasNextBoolean())
               scanner.next();
            set(booleanKey, scanner.nextBoolean());
         }
      }
      scanner.close();
   }

   @Override
   public String toString()
   {
      List<StoredPropertyKey<?>> storedPropertyKeys = keys.keys();
      StringBuilder result = new StringBuilder();
      for (int i = 0; i < storedPropertyKeys.size(); i++)
      {
         StoredPropertyKey<?> key = storedPropertyKeys.get(i);
         result.append(key.getCamelCasedName());
         result.append(": ");
         result.append(serializeValue(get(key)));
         if (i < storedPropertyKeys.size() - 1)
         {
            result.append(", ");
         }
      }
      return result.toString();
   }

   @Override
   public String getCurrentVersionSuffix()
   {
      return currentVersionSuffix;
   }

   public String getCapitalizedClassName()
   {
      return capitalizedClassName;
   }

   @Override
   public String getTitle()
   {
      return title;
   }

   @Override
   public void setTitle(String title)
   {
      this.title = title;
   }
}
