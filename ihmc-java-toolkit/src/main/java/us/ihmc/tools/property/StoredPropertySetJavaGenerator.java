package us.ihmc.tools.property;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.BooleanNode;
import com.fasterxml.jackson.databind.node.DoubleNode;
import com.fasterxml.jackson.databind.node.IntNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.string.StringTools;

import java.util.ArrayList;

/**
 * To create a StoredPropertySet, create a main, us.ihmc.YourStoredPropertySet.java
 *
 * <pre>
 * public static void main(String[] args)
 * {
 *    StoredPropertySetJavaGenerator generator = new StoredPropertySetJavaGenerator(StoredPropertySetGeneratorTest.class,
 *                                                                                  "ihmc-open-robotics-software",
 *                                                                                  "ihmc-java-toolkit/src/test/resources",
 *                                                                                  "ihmc-java-toolkit/src/test/java");
 *    generator.generate();
 * }
 * </pre>
 *
 * where the paths are replaced to match your situation. The subsequest paths may be shorter or longer depending on how nested the
 * projects are. Typically, the "directory name to assume present" is the name of the repository. These paths are necessary
 * to allow saving the parameters in version control.
 *
 * Then, create a us.ihmc.YourStoredPropertySetName.json in the resources folder. The name should be the exact same as the *.java class.
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
 * Run the main, and then you will be further assisted by the generated code there.
 */
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
   private record StoredPropertyFromFile(String titleCasedName, String typeName, String typePrimitiveName) { }
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

   public void generate()
   {
      JSONFileTools.loadFromClasspath(clazz, jsonFileName, node ->
      {
         if (node instanceof ObjectNode objectNode)
         {
            objectNode.fieldNames().forEachRemaining(fieldName ->
            {
               JsonNode propertyNode = objectNode.get(fieldName);
               LogTools.info("Name: {} Value: {}", fieldName, propertyNode);
               if (fieldName.equals("title"))
               {
                  storedPropertySetTitle = propertyNode.asText();
               }
               else
               {
                  if (propertyNode instanceof BooleanNode booleanNode)
                  {
                     storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Boolean", "boolean"));
                  }
                  else if (propertyNode instanceof DoubleNode doubleNode)
                  {
                     storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Double", "double"));
                  }
                  else if (propertyNode instanceof IntNode integerNode)
                  {
                     storedPropertiesFromFile.add(new StoredPropertyFromFile(fieldName, "Integer", "int"));
                  }
               }
            });
         }
      });

      String primaryJavaFileContents =
      """
      package %s;
      
      import us.ihmc.tools.property.*;
      
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
            StoredPropertySetJavaGenerator generator = new StoredPropertySetJavaGenerator(%2$s.class,
                                                                                          DIRECTORY_NAME_TO_ASSUME_PRESENT,
                                                                                          SUBSEQUENT_PATH_TO_RESOURCE_FOLDER,
                                                                                          SUBSEQUENT_PATH_TO_JAVA_FOLDER);
            generator.generate();
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
}
