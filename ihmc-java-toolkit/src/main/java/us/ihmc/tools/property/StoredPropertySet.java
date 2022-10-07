package us.ihmc.tools.property;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.fasterxml.jackson.databind.node.TextNode;
import org.apache.commons.lang3.StringUtils;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

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
 *    StoredPropertySet parameters = new StoredPropertySet(keys,
 *                                                         YourStoredPropertySet.class,
 *                                                         DIRECTORY_NAME_TO_ASSUME_PRESENT,
 *                                                         SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
 *    parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
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
   private Class<?> classForLoading;
   private String directoryNameToAssumePresent;
   private String subsequentPathToResourceFolder;
   private final WorkspaceDirectory workspaceDirectory;
   private final String uncapitalizedClassName;
   private final String capitalizedClassName;
   private WorkspaceFile workspaceLegacyINIFile;
   private WorkspaceFile workspaceJSONFile;

   private final Map<StoredPropertyKey, List<Runnable>> propertyChangedListeners = new HashMap<>();

   public StoredPropertySet(StoredPropertyKeyList keys,
                            Class<?> classForLoading,
                            String directoryNameToAssumePresent,
                            String subsequentPathToResourceFolder)
   {
      this(keys, classForLoading, directoryNameToAssumePresent, subsequentPathToResourceFolder, "");
   }

   public StoredPropertySet(StoredPropertyKeyList keys,
                            Class<?> classForLoading,
                            String directoryNameToAssumePresent,
                            String subsequentPathToResourceFolder,
                            String versionSuffix)
   {
      this.keys = keys;
      this.uncapitalizedClassName = StringUtils.uncapitalize(classForLoading.getSimpleName());
      this.capitalizedClassName = classForLoading.getSimpleName();
      this.classForLoading = classForLoading;
      title = classForLoading.getSimpleName();
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
      workspaceDirectory = new WorkspaceDirectory(directoryNameToAssumePresent, subsequentPathToResourceFolder, classForLoading);

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

   public void generateJavaFiles(String subsequentPathToJavaFolder)
   {
      StoredPropertySetJavaGenerator generator = new StoredPropertySetJavaGenerator(classForLoading,
                                                                                    directoryNameToAssumePresent,
                                                                                    subsequentPathToResourceFolder,
                                                                                    subsequentPathToJavaFolder);
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
      return (Double) values[key.getIndex()];
   }

   @Override
   public int get(IntegerStoredPropertyKey key)
   {
      return (Integer) values[key.getIndex()];
   }

   @Override
   public boolean get(BooleanStoredPropertyKey key)
   {
      return (Boolean) values[key.getIndex()];
   }

   /**
    * @return the value or null
    */
   @Override
   public <T> T get(StoredPropertyKey<T> key)
   {
      return (T) values[key.getIndex()];
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

   public void updateBackingSaveFile(String versionSuffix)
   {
      currentVersionSuffix = versionSuffix;
      legacyFileNameINI = uncapitalizedClassName + currentVersionSuffix + ".ini";
      workspaceLegacyINIFile = new WorkspaceFile(workspaceDirectory, legacyFileNameINI);
      saveFileNameJSON = classForLoading.getSimpleName() + currentVersionSuffix + ".json";
      workspaceJSONFile = new WorkspaceFile(workspaceDirectory, saveFileNameJSON);
   }

   @Override
   public void load()
   {
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
                     key.setDescription(arrayNode.get(1).asText());
                  }
                  else
                  {
                     stringValue = propertyNode.asText();
                  }

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
         });
      }
      else if (iniResourceExists()) // fallback to the old INI format
      {
         ExceptionTools.handle(() ->
         {
            Properties properties = new Properties();
            InputStream streamForLoading = workspaceLegacyINIFile.getClasspathResourceAsStream();

            LogTools.info("Loading parameters from {}", legacyFileNameINI);
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
      LogTools.info("Saving parameters to {}", fileForSaving.getFileName());
      if (workspaceDirectory.isFileAccessAvailable())
      {
         FileTools.ensureDirectoryExists(workspaceDirectory.getDirectoryPath(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }
      JSONFileTools.save(fileForSaving, jsonRootObjectNode ->
      {
         jsonRootObjectNode.put("title", title);
         for (StoredPropertyKey<?> key : keys.keys())
         {
            if (!key.getDescription().isEmpty())
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
         FileTools.deleteQuietly(workspaceLegacyINIFile.getFilePath());
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
         return workspaceDirectory.getDirectoryPath();
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

   public String getCurrentVersionSuffix()
   {
      return currentVersionSuffix;
   }

   public String getCapitalizedClassName()
   {
      return capitalizedClassName;
   }

   public String getTitle()
   {
      return title;
   }
}