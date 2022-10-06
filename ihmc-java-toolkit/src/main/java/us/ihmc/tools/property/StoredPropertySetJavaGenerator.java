package us.ihmc.tools.property;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.*;
import org.apache.commons.text.WordUtils;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.string.StringTools;

import java.util.ArrayList;

public class StoredPropertySetJavaGenerator
{
   private final String jsonFileName;
   private final Class<?> clazz;
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

   public StoredPropertySetJavaGenerator(Class<?> clazz,
                                         String directoryNameToAssumePresent,
                                         String subsequentPathToResourceFolder,
                                         String subsequentPathToJavaFolder)
   {
      this.clazz = clazz;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
      this.subsequentPathToJavaFolder = subsequentPathToJavaFolder;

      javaDirectory = new WorkspaceDirectory(directoryNameToAssumePresent, subsequentPathToJavaFolder, clazz);
      jsonFileName = clazz.getSimpleName() + ".json";
      primaryJavaFile = new WorkspaceFile(javaDirectory, clazz.getSimpleName() + ".java");
      basicsJavaFile = new WorkspaceFile(javaDirectory, clazz.getSimpleName() + "Basics.java");
      readOnlyJavaFile = new WorkspaceFile(javaDirectory, clazz.getSimpleName() + "ReadOnly.java");
   }

   public void loadFromJSON()
   {
      JSONFileTools.loadFromClasspath(clazz, jsonFileName, node ->
      {
         if (node instanceof ObjectNode objectNode)
         {
            objectNode.fieldNames().forEachRemaining(fieldName ->
            {
               JsonNode propertyNode = objectNode.get(fieldName);
               if (fieldName.equals("title"))
               {
                  storedPropertySetTitle = propertyNode.asText();
               }
               else
               {
                  String description = "";
                  if (propertyNode instanceof ArrayNode arrayNode)
                  {
                     description = arrayNode.get(1).asText();
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
       * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
       * and run the main to regenerate.
       */
      public class %2$s extends StoredPropertySet implements %2$sBasics
      {
         public static final String DIRECTORY_NAME_TO_ASSUME_PRESENT = "%4$s";
         public static final String SUBSEQUENT_PATH_TO_RESOURCE_FOLDER = "%5$s";
         public static final String SUBSEQUENT_PATH_TO_JAVA_FOLDER = "%6$s";
         
         public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();
         
      %3$s
         public %2$s()
         {
            super(keys, %2$s.class, DIRECTORY_NAME_TO_ASSUME_PRESENT, SUBSEQUENT_PATH_TO_RESOURCE_FOLDER);
            load();
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
      """.formatted(clazz.getPackage().getName(),
                    clazz.getSimpleName(),
                    getParameterKeysStrings(),
                    directoryNameToAssumePresent,
                    subsequentPathToResourceFolder,
                    subsequentPathToJavaFolder);

      FileTools.write(primaryJavaFile.getFilePath(), primaryJavaFileContents.getBytes(), WriteOption.TRUNCATE, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

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
      """.formatted(clazz.getPackage().getName(), clazz.getSimpleName(), getParameterSetterStrings());

      FileTools.write(basicsJavaFile.getFilePath(), basicsJavaFileContents.getBytes(), WriteOption.TRUNCATE, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

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
      """.formatted(clazz.getPackage().getName(), clazz.getSimpleName(), getParameterGetterStrings(), clazz.getPackage().getName());

      FileTools.write(readOnlyJavaFile.getFilePath(),
                      readOnlyJavaFileContents.getBytes(),
                      WriteOption.TRUNCATE,
                      DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   private String getParameterKeysStrings()
   {
      StringBuilder propertyKeyDeclarations = new StringBuilder();
      for (StoredPropertyFromFile storedPropertyFromFile : storedPropertiesFromFile)
      {
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
                                    clazz.getSimpleName())
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
                                    clazz.getSimpleName())
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
         """.indent(3).formatted(WordUtils.wrap(description, 80, "\n    * ", true));
      }
   }
}
