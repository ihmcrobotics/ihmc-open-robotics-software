package us.ihmc.tools.property;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.*;
import org.apache.commons.text.WordUtils;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.string.StringTools;

import java.util.ArrayList;

public class StoredPropertySetJavaGenerator
{
   private final String jsonFileName;
   private final Class<?> basePropertySetClass;
   private String directoryNameToAssumePresent;
   private String subsequentPathToResourceFolder;
   private String subsequentPathToJavaFolder;
   private final WorkspaceDirectory javaDirectory;
   private final WorkspaceFile primaryJavaFile;
   private final WorkspaceFile basicsJavaFile;
   private final WorkspaceFile readOnlyJavaFile;
   private String storedPropertySetTitle;
   private record StoredPropertyFromFile(String titleCasedName, String typeName, String typePrimitiveName, String description) { }
   private final ArrayList<StoredPropertyFromFile> storedPropertiesFromFile = new ArrayList<>();

   public StoredPropertySetJavaGenerator(Class<?> basePropertySetClass,
                                         String directoryNameToAssumePresent,
                                         String subsequentPathToResourceFolder,
                                         String subsequentPathToJavaFolder)
   {
      this.basePropertySetClass = basePropertySetClass;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
      this.subsequentPathToJavaFolder = subsequentPathToJavaFolder;

      javaDirectory = new WorkspaceDirectory(directoryNameToAssumePresent, subsequentPathToJavaFolder, basePropertySetClass);
      jsonFileName = basePropertySetClass.getSimpleName() + ".json";
      primaryJavaFile = new WorkspaceFile(javaDirectory, basePropertySetClass.getSimpleName() + ".java");
      basicsJavaFile = new WorkspaceFile(javaDirectory, basePropertySetClass.getSimpleName() + "Basics.java");
      readOnlyJavaFile = new WorkspaceFile(javaDirectory, basePropertySetClass.getSimpleName() + "ReadOnly.java");
   }

   public void loadFromJSON()
   {
      JSONFileTools.loadFromClasspath(basePropertySetClass, jsonFileName, node ->
      {
         if (node instanceof ObjectNode objectNode)
         {
            objectNode.fieldNames().forEachRemaining(fieldName ->
            {
               JsonNode propertyNode = objectNode.get(fieldName);
               if (fieldName.equals("title"))
               {
                  storedPropertySetTitle = propertyNode.textValue();
               }
               else
               {
                  String description = "";
                  if (propertyNode instanceof ArrayNode arrayNode)
                  {
                     description = arrayNode.get(1).textValue();
                     if (arrayNode.get(0) instanceof BooleanNode)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Boolean", "boolean", description));
                     }
                     else if (arrayNode.get(0) instanceof DoubleNode)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Double", "double", description));
                     }
                     else if (arrayNode.get(0) instanceof IntNode)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Integer", "int", description));
                     }
                  }
                  else if (propertyNode instanceof ObjectNode keyObjectNode)
                  {

                     JsonNode descriptionNode = keyObjectNode.get("description");
                     if (descriptionNode != null)
                        description = descriptionNode.textValue();

                     JsonNode valueNode = keyObjectNode.get("value");
                     JsonNode typeNode = keyObjectNode.get("type");

                     // Here we are supporting JSON that doesn't define values;
                     // It's not necessary for generation and supports having values only specified when needed
                     boolean isBooleanProperty = valueNode instanceof BooleanNode;
                     isBooleanProperty |= typeNode != null && typeNode.textValue().equals("Boolean");

                     boolean isDoubleProperty = valueNode instanceof DoubleNode;
                     isDoubleProperty |= typeNode != null && typeNode.textValue().equals("Double");

                     boolean isIntegerProperty = valueNode instanceof IntNode;
                     isIntegerProperty |= typeNode != null && typeNode.textValue().equals("Integer");

                     if (isBooleanProperty)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Boolean", "boolean", description));
                     }
                     else if (isDoubleProperty)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Double", "double", description));
                     }
                     else if (isIntegerProperty)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Integer", "int", description));
                     }
                  }
                  else
                  {
                     if (propertyNode instanceof BooleanNode)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Boolean", "boolean", description));
                     }
                     else if (propertyNode instanceof DoubleNode)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Double", "double", description));
                     }
                     else if (propertyNode instanceof IntNode)
                     {
                        storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Integer", "int", description));
                     }
                  }
               }
            });
         }
      });
   }

   public void loadFromStoredPropertySet(StoredPropertySet storedPropertySet)
   {
      storedPropertySetTitle = storedPropertySet.getTitle();
      for (StoredPropertyKey<?> key : storedPropertySet.getKeyList().keys())
      {
         if (key instanceof BooleanStoredPropertyKey)
         {
            storedPropertiesFromFile.add(new StoredPropertyFromFile(key.getTitleCasedName(), "Boolean", "boolean", key.getDescription()));
         }
         else if (key instanceof DoubleStoredPropertyKey)
         {
            storedPropertiesFromFile.add(new StoredPropertyFromFile(key.getTitleCasedName(), "Double", "double", key.getDescription()));
         }
         else if (key instanceof IntegerStoredPropertyKey)
         {
            storedPropertiesFromFile.add(new StoredPropertyFromFile(key.getTitleCasedName(), "Integer", "int", key.getDescription()));
         }
      }
   }

   public void generate()
   {
      // This code uses text blocks. Read about them at https://docs.oracle.com/en/java/javase/17/text-blocks/index.html
      // We are using the formatted method which is just the using regular Java formatter:
      // https://docs.oracle.com/en/java/javase/17/docs/api/java.base/java/util/Formatter.html
      // We are using the [argument_index$] to reuse variables within the text block.
      String primaryJavaFileContents =
      """
            package %s;
                  
            import us.ihmc.tools.property.*;
                  
            /**
             * The JSON file for this property set is located here:
             * %5$s/%7$s/%2$s.json
             *
             * This class was auto generated. Property attributes must be edited in the JSON file,
             * after which this class should be regenerated by running the main. This class uses
             * the generator to assist in the addition, removal, and modification of property keys.
             * It is permissible to forgo these benefits and abandon the generator, in which case
             * you should also move it from the generated-java folder to the java folder.
             *
             * If the constant paths have changed, change them in this file and run the main to regenerate.
             */
            public class %2$s extends StoredPropertySet implements %2$sBasics
            {
               public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "%4$s";
               public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "%5$s";
               public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "%6$s";
               
               public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();
               
            %3$s
               /**
                * Loads this property set.
                */
               public %2$s()
               {
                  this("");
               }
               
               /**
                * Loads an alternate version of this property set in the same folder.
                */
               public %2$s(String versionSpecifier)
               {
                  this(%2$s.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, versionSpecifier);
               }
               
               /**
                * Loads an alternate version of this property set in other folders.
                */
               public %2$s(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder, String versionSuffix)
               {
                  super(keys, classForLoading, %2$s.class, directoryNameToAssumePresent, subsequentPathToResourceFolder, versionSuffix);
                  load();
               }
               
               public %2$s(StoredPropertySetReadOnly other)
               {
                  super(keys, %2$s.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER, other.getCurrentVersionSuffix());
                  set(other);
               }
                  
               public static void main(String[] args)
               {
                  StoredPropertySet parameters = new StoredPropertySet(keys,
                                                                       %2$s.class,
                                                                       DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                                       SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
                  parameters.generateJavaFiles(SUBSEQUENT_PATH_TO_JAVA_FOLDER);
               }
            }
            """.formatted(basePropertySetClass.getPackage().getName(),
                          basePropertySetClass.getSimpleName(),
                          getParameterKeysStrings(),
                          directoryNameToAssumePresent,
                          subsequentPathToResourceFolder,
                          subsequentPathToJavaFolder,
                          basePropertySetClass.getPackage().getName().replaceAll("\\.", "/"));

      FileTools.write(primaryJavaFile.getFilePath(), primaryJavaFileContents.getBytes(), WriteOption.TRUNCATE, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      LogTools.info("Generated successfully: {}", primaryJavaFile.getFilePath());

      String basicsJavaFileContents =
      """
      package %s;
      
      import us.ihmc.tools.property.StoredPropertySetBasics;
      
      /**
       * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
       * and run the main in super to regenerate.
       */
      public interface %2$sBasics extends %2$sReadOnly, StoredPropertySetBasics
      {
      %3$s}
      """.formatted(basePropertySetClass.getPackage().getName(), basePropertySetClass.getSimpleName(), getParameterSetterStrings());

      FileTools.write(basicsJavaFile.getFilePath(), basicsJavaFileContents.getBytes(), WriteOption.TRUNCATE, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      LogTools.info("Generated successfully: {}", basicsJavaFile.getFilePath());

      String readOnlyJavaFileContents =
      """
      package %s;
      
      import us.ihmc.tools.property.StoredPropertySetReadOnly;
      
      import static %4$s.%2$s.*;
      
      /**
       * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
       * and run the main in super to regenerate.
       */
      public interface %2$sReadOnly extends StoredPropertySetReadOnly
      {
      %3$s}
      """.formatted(basePropertySetClass.getPackage().getName(), basePropertySetClass.getSimpleName(), getParameterGetterStrings(), basePropertySetClass.getPackage().getName());

      FileTools.write(readOnlyJavaFile.getFilePath(),
                      readOnlyJavaFileContents.getBytes(),
                      WriteOption.TRUNCATE,
                      DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      LogTools.info("Generated successfully: {}", readOnlyJavaFile.getFilePath());
   }

   private String getParameterKeysStrings()
   {
      StringBuilder propertyKeyDeclarations = new StringBuilder();
      for (StoredPropertyFromFile storedPropertyFromFile : storedPropertiesFromFile)
      {
         propertyKeyDeclarations.append(getParameterJavadoc(storedPropertyFromFile.description()));
         propertyKeyDeclarations.append(
            """
            public static final %2$sStoredPropertyKey %1$s = keys.add%2$sKey("%3$s");
            """.indent(3).formatted(StringTools.titleToCamelCase(storedPropertyFromFile.titleCasedName()),
                                    storedPropertyFromFile.typeName(),
                                    storedPropertyFromFile.titleCasedName())
         );
      }
      return propertyKeyDeclarations.toString();
   }

   private String getParameterSetterStrings()
   {
      StringBuilder propertyKeyDeclarations = new StringBuilder();
      for (int i = 0; i < storedPropertiesFromFile.size(); i++)
      {
         StoredPropertyFromFile storedPropertyFromFile = storedPropertiesFromFile.get(i);
         propertyKeyDeclarations.append(getParameterJavadoc(storedPropertyFromFile.description()));
         propertyKeyDeclarations.append(
            """
            default void set%1$s(%2$s %3$s)
            {
               set(%4$s.%3$s, %3$s);
            }
            """.indent(3).formatted(StringTools.titleToPascalCase(storedPropertyFromFile.titleCasedName()),
                                    storedPropertyFromFile.typePrimitiveName(),
                                    StringTools.titleToCamelCase(storedPropertyFromFile.titleCasedName()),
                                    basePropertySetClass.getSimpleName())
         );
         if (i < storedPropertiesFromFile.size() - 1)
         {
            propertyKeyDeclarations.append("\n");
         }
      }
      return propertyKeyDeclarations.toString();
   }

   private String getParameterGetterStrings()
   {
      StringBuilder propertyKeyDeclarations = new StringBuilder();
      for (int i = 0; i < storedPropertiesFromFile.size(); i++)
      {
         StoredPropertyFromFile storedPropertyFromFile = storedPropertiesFromFile.get(i);
         propertyKeyDeclarations.append(getParameterJavadoc(storedPropertyFromFile.description()));
         propertyKeyDeclarations.append(
            """
            default %2$s get%1$s()
            {
               return get(%3$s);
            }
            """.indent(3).formatted(StringTools.titleToPascalCase(storedPropertyFromFile.titleCasedName()),
                                    storedPropertyFromFile.typePrimitiveName(),
                                    StringTools.titleToCamelCase(storedPropertyFromFile.titleCasedName()),
                                    basePropertySetClass.getSimpleName())
         );
         if (i < storedPropertiesFromFile.size() - 1)
         {
            propertyKeyDeclarations.append("\n");
         }
      }
      return propertyKeyDeclarations.toString();
   }

   private String getParameterJavadoc(String description)
   {
      if (description.isEmpty())
      {
         return "";
      }
      else
      {
         return
         """
         /**
          * %s
          */
         """.indent(3).formatted(WordUtils.wrap(description, 80, "\n    * ", false));
      }
   }
}
