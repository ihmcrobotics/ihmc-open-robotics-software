package us.ihmc.utilities.ros;

import org.apache.commons.lang3.text.WordUtils;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.ros.generators.*;
import us.ihmc.utilities.ros.msgToPacket.converter.GenericROSTranslationTools;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

public class ROSMessageFileCreator
{
   private static final int WRAP_LENGTH = 100;
   boolean overwriteSubMessages;

   public ROSMessageFileCreator(boolean overwriteSubMessages)
   {
      this.overwriteSubMessages = overwriteSubMessages;
   }

   public String createNewRosMessageFromGenerator(RosCustomGenerator generator, boolean overwrite) throws Exception
   {
      String messageName = generator.getMessageName();
      Path msgDirectoryPath = Paths.get("..", "IHMCROSTools", "ROSMessagesAndServices", generator.getRosPackage(), "msg");
      if(!Files.exists(msgDirectoryPath))
      {
         Files.createDirectories(msgDirectoryPath);
      }

      File messageFile = msgDirectoryPath.resolve(messageName + ".msg").toFile();

      if (overwrite ||  !messageFile.exists())
      {
         StringBuilder fileContents = new StringBuilder();
         try(FileWriter writer = new FileWriter(messageFile))
         {
            addTypeDocumentationToFileContents(generator.getTypeDocumentation(), messageName, fileContents);

            for (RosFieldDefinition rosFieldDefinition : generator.getFields())
            {
               cleanupAndLineWrapDocumentation(fileContents, rosFieldDefinition.getDocumentation());
               fileContents.append(rosFieldDefinition.getType()).append(" ").append(rosFieldDefinition.getFieldName()); //.append("\n\n");
               if(rosFieldDefinition.isConstant())
               {
                  fileContents.append("=").append(rosFieldDefinition.getConstantValue());
               }

               fileContents.append("\n\n");
            }

            writer.append(fileContents.toString());
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
      return messageName;
   }

   @SuppressWarnings("unchecked")
   public String createNewRosMessage(Class clazz, boolean overwrite) throws Exception
   {
      RosMessagePacket rosMessageAnnotation = (RosMessagePacket) clazz.getAnnotation(RosMessagePacket.class);
      String messageName = clazz.getSimpleName();

      Path msgDirectoryPath = Paths.get("..", "IHMCROSTools", "ROSMessagesAndServices", rosMessageAnnotation.rosPackage(), "msg");
      if(!Files.exists(msgDirectoryPath))
      {
         Files.createDirectories(msgDirectoryPath);
      }

      messageName = GenericROSTranslationTools.getRosMessageClassNameFromIHMCMessage(messageName);

      File messageFile = msgDirectoryPath.resolve(messageName + ".msg").toFile();

      if (overwrite ||  !messageFile.exists())
      {
         messageFile.delete();

         try(FileWriter writer = new FileWriter(messageFile))
         {
            messageFile.createNewFile();
//            PrintTools.info("Created empty msg file: " + messageFile.getCanonicalPath());

            StringBuilder fileContents = new StringBuilder();

            addTypeDocumentationToFileContents(rosMessageAnnotation.documentation(), messageName, fileContents);

            fileContents.append("\n");

            ArrayList<Field> exportedFields = new ArrayList<>();
            Set<Class<? extends Enum>> exportedEnumClasses = new HashSet<>();

            gatherExportedFieldsAndEnums(clazz, exportedFields, exportedEnumClasses);

            addFieldsToFileContents(fileContents, exportedFields);

            fileContents.append("\n");

            addEnumDocumentationToFileContents(fileContents, exportedEnumClasses);

            writer.write(fileContents.toString());
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      return messageName;
   }

   private void gatherExportedFieldsAndEnums(Class clazz, ArrayList<Field> exportedFields, Set<Class<? extends Enum>> exportedEnumClasses)
   {
      for (Field field : clazz.getFields())
      {
         if(field.isAnnotationPresent(RosExportedField.class))
         {
            if(Enum.class.isAssignableFrom(field.getType()))
            {
               exportedEnumClasses.add((Class<? extends Enum>) field.getType());
            }

            exportedFields.add(field);
         }
      }
   }

   private void addFieldsToFileContents(StringBuilder fileContents, Collection<Field> exportedFields) throws IOException
   {
      for (Field exportedField : exportedFields)
      {
         cleanupAndLineWrapDocumentation(fileContents, exportedField.getAnnotation(RosExportedField.class).documentation());
         String rosTypeForJavaType = GenericROSTranslationTools.getRosTypeForJavaType(exportedField, exportedField.getType());
         System.out.println("Received ros type of " + rosTypeForJavaType + " for field " + exportedField.getName() + " declared in " + exportedField
               .getDeclaringClass().getSimpleName());
         fileContents.append(rosTypeForJavaType).append(" ").append(camelCaseToLowerCaseWithUnderscores(exportedField.getName())).append("\n\n");
      }
   }

   @SuppressWarnings("unchecked")
   private void addEnumDocumentationToFileContents(StringBuilder fileContents, Set<Class<? extends Enum>> exportedEnumClasses) throws IOException
   {
      if(!exportedEnumClasses.isEmpty())
      {
         fileContents.append("# This message utilizes \"enums\". Enum value information for this message follows.").append("\n\n");
      }

      for (Class<? extends Enum> exportedEnumClass : exportedEnumClasses)
      {

         fileContents.append("# \"").append(camelCaseToLowerCaseWithUnderscores(exportedEnumClass.getSimpleName())).append("\" enum values:").append("\n");

         ArrayList<Field> documentedFields = getExplicitlyDocumentedEnumConstants(exportedEnumClass);

         if(!documentedFields.isEmpty())
         {
            addExplicitlyDocumentedEnumToFileContents(fileContents, exportedEnumClass, documentedFields);
         }
         else if(GenericROSTranslationTools.hasDocumentation(exportedEnumClass))
         {
            addImplicitlyDocumentedEnumToFileContents(fileContents, exportedEnumClass);
         }
         else
         {
            PrintTools.error(ROSMessageFileCreator.class, "Could not find any documentation for the enum values of " + exportedEnumClass.getCanonicalName()
            + " even though it is ROS Exported Field. Excluding from generated files. Please add documentation to the enum values. If this is a low-level/implicitly documented enum, consult ROSMessageConversionTools.java's getDocumentation() method.");
         }

         fileContents.append("\n");
      }
   }

   private ArrayList<Field> getExplicitlyDocumentedEnumConstants(Class<? extends Enum> exportedEnumClass)
   {
      ArrayList<Field> documentedFields = new ArrayList<>();
      for (Field field : exportedEnumClass.getFields())
      {
         if(field.isEnumConstant() && field.isAnnotationPresent(RosEnumValueDocumentation.class))
         {
            documentedFields.add(field);
         }
      }
      return documentedFields;
   }

   private void addImplicitlyDocumentedEnumToFileContents(StringBuilder fileContents, Class<? extends Enum> exportedEnumClass)
   {
      Enum[] enumConstants = exportedEnumClass.getEnumConstants();
      for (Enum enumConstant : enumConstants)
      {
         fileContents.append("uint8 ").append(enumConstant.name()).append("=");
         String enumDocumentation = GenericROSTranslationTools.getDocumentation(exportedEnumClass, enumConstant);
         fileContents.append(enumConstant.ordinal()).append(" ");
         fileContents.append("# ").append(enumDocumentation).append("\n");
      }
   }

   private void addExplicitlyDocumentedEnumToFileContents(StringBuilder fileContents, Class<? extends Enum> exportedEnumClass,
         ArrayList<Field> documentedFields)
   {
      for (Field documentedField : documentedFields)
      {
         fileContents.append("uint8 ").append(documentedField.getName()).append("=");
         for (Enum anEnum : exportedEnumClass.getEnumConstants())
         {
            if(anEnum.name().equals(documentedField.getName()))
            {
               RosEnumValueDocumentation annotation = documentedField.getAnnotation(RosEnumValueDocumentation.class);
               fileContents.append(anEnum.ordinal()).append(" ");
               fileContents.append("# ").append(annotation.documentation()).append("\n");
               break;
            }
         }
      }
   }

   private void addTypeDocumentationToFileContents(String typeDocumentation, String messageName, StringBuilder fileContents)
   {
      fileContents.append("## ").append(messageName).append("\n");

      cleanupAndLineWrapDocumentation(fileContents, typeDocumentation);
   }

   private void cleanupAndLineWrapDocumentation(StringBuilder fileContents, String documentation)
   {
      String[] documentationLines = WordUtils.wrap(documentation.replace("\n", " ").replace("\r", " ").replace("\r\n", " "), WRAP_LENGTH).split(System.lineSeparator());

      for (String line : documentationLines)
      {
         fileContents.append("# ").append(line).append("\n");
      }
   }

   private String camelCaseToLowerCaseWithUnderscores(String in)
   {
      String regex = "([a-z])([A-Z]+)";
      String replacement = "$1_$2";

      String inWithUnderscores = in.replaceAll(regex, replacement);
      String ret = inWithUnderscores.toLowerCase();

      return ret;
   }
}
