package us.ihmc.utilities.ros;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.text.WordUtils;
import us.ihmc.communication.annotations.ros.RosMessagePacket;
import us.ihmc.communication.annotations.ros.RosExportedField;
import us.ihmc.communication.annotations.ros.RosIgnoredField;
import us.ihmc.tools.io.printing.PrintTools;

public class ROSMessageGenerator
{
   boolean overwriteSubMessages;

   public ROSMessageGenerator(boolean overwriteSubMessages)
   {
      this.overwriteSubMessages = overwriteSubMessages;
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

      if(!messageName.endsWith("Message"))
      {
          messageName += "Message";
      }

      messageName = StringUtils.replace(messageName, "Message", "RosMessage");

      File messageFile = msgDirectoryPath.resolve(messageName + ".msg").toFile();

      if (overwrite ||  !messageFile.exists())
      {
         messageFile.delete();

         try(FileWriter writer = new FileWriter(messageFile))
         {
            messageFile.createNewFile();
            PrintTools.info("Created empty msg file: " + messageFile.getCanonicalPath());

            StringBuilder fileContents = new StringBuilder();

            fileContents.append("## ").append(messageName).append("\n");

            String classDocumentation = rosMessageAnnotation.documentation();

            cleanupAndLineWrapDocumentation(fileContents, classDocumentation);

            fileContents.append("\n");

            Field[] fields = clazz.getFields();
            Set<String> enumsAlreadyDocumented = new TreeSet<String>();
            for (Field field : fields)
            {
//               if(Enum.class.isAssignableFrom(field.getType()) && RosMessageGenerationTools.isDocumented((Class<? extends Enum<?>>) field.getType()))
//               {
//                  addEnumFieldToFileContents(field, fileContents);
//               }
               if (isConstant(field) || isIgnoredField(field))
               {
                  continue;
               }

               if (field.getType().isEnum())
               {
                  String enumName = field.getType().getSimpleName();
                  boolean duplicateEnum = enumsAlreadyDocumented.contains(enumName);
                  if (!duplicateEnum)
                  {
                     enumsAlreadyDocumented.add(enumName);
                  }

//                  outBuffer += printType(field, duplicateEnum);
               }
               else
               {
//                  outBuffer += printType(field);
               }

               String formattedFieldName = camelCaseToLowerCaseWithUnderscores(field.getName());
//               outBuffer += " " + formattedFieldName + "\n" + "\n";
            }

//            writer.write(fileContents.toString());
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      return messageName;
   }

   @SuppressWarnings("unchecked")
   private void addEnumFieldToFileContents(Field field, StringBuilder fileContents)
   {
//      DocumentedEnum documentedEnum = RosMessageGenerationTools.getDocumentation((Class<? extends Enum<?>>) field.getType());
//      Object[] documentedValues = documentedEnum.getDocumentedValues();

//      fileContents.append("# Options for ").append(field.getName()).append("\n");

//      for (int i = 0; i < documentedValues.length; i++)
//      {
//         buffer +=
//         if (duplicateEnum)
//         {
//            buffer += "# ";
//         }
//         else
//         {
//            buffer += "uint8 ";
//         }
//         buffer += documentedValues[i].toString() + "=" + i;
//
//         String documentation = documentedEnum.getDocumentation(documentedValues[i]);
//         buffer += " # " + documentation;
//         buffer += "\n";
//      }
//
//      buffer += "uint8";
   }

   private void cleanupAndLineWrapDocumentation(StringBuilder fileContents, String documentation)
   {
      String[] documentationLines = WordUtils.wrap(documentation.replace("\n", " ").replace("\r", " ").replace("\r\n", " "), 78).split(System.lineSeparator());

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

   private boolean isIgnoredField(Field field)
   {
      return field.isAnnotationPresent(RosIgnoredField.class);
   }

   private boolean isConstant(Field field)
   {
      int modifier = field.getModifiers();

      return Modifier.isFinal(modifier) && Modifier.isPublic(modifier) && Modifier.isStatic(modifier);
   }

   private String printType(Field field) throws Exception
   {
      return printType(field, false);
   }

   private String printType(Field field, boolean duplicateEnum) throws Exception
   {
      String buffer = "";
      RosExportedField fieldAnnotation = field.getAnnotation(RosExportedField.class);
      if (fieldAnnotation != null)
      {
         String[] annotationString = fieldAnnotation.documentation().split("\r?\n|\r");
         for (String line : annotationString)
         {
            buffer += "# " + line + "\n";
         }
      }

      if (List.class.isAssignableFrom(field.getType()))
      {
         Class genericType = ((Class) ((ParameterizedType) field.getGenericType()).getActualTypeArguments()[0]);
         buffer += printType(genericType, field.getName(), duplicateEnum);
         buffer += "[]";
      }
      else
      {
         buffer += printType(field.getType(), field.getName(), duplicateEnum);
      }

      return buffer;
   }

   @SuppressWarnings("unchecked")
   private String printType(Class clazz, String varName) throws Exception
   {
      return printType(clazz, varName, false);
   }

   @SuppressWarnings("unchecked")
   private String printType(Class clazz, String varName, boolean duplicateEnum) throws Exception
   {
      String buffer = "";
      if (clazz == null)
      {
         return buffer;
      }

      if (clazz.isArray())
      {
         buffer += printType(clazz.getComponentType(), varName, duplicateEnum);
         buffer += "[]";
      }
      else if (clazz.isEnum())
      {
//         DocumentedEnum documentedEnum = RosMessageGenerationTools.getDocumentation(clazz);
         if(false)
         {
//            Object[] documentedValues = documentedEnum.getDocumentedValues();
//
//            buffer += "# Options for " + varName + "\n";
//
//            for (int i = 0; i < documentedValues.length; i++)
//            {
//               if (duplicateEnum)
//               {
//                  buffer += "# ";
//               }
//               else
//               {
//                  buffer += "uint8 ";
//               }
//               buffer += documentedValues[i].toString() + "=" + i;
//
//               String documentation = documentedEnum.getDocumentation(documentedValues[i]);
//               buffer += " # " + documentation;
//               buffer += "\n";
//            }

            buffer += "uint8";
         }
         else
         {
            System.err.println(clazz.getSimpleName() + " is not a DocumentedEnum and not defined RosMessageGenerationTools! Fix and rerun!");
         }

      }
      else if (clazz.equals(byte.class) || clazz.equals(Byte.class))
      {
         buffer += "int8";
      }
      else if (clazz.equals(short.class) || clazz.equals(Short.class))
      {
         buffer += "int16";
      }
      else if (clazz.equals(int.class) || clazz.equals(Integer.class))
      {
         buffer += "int32";
      }
      else if (clazz.equals(long.class) || clazz.equals(Long.class))
      {
         buffer += "int64";
      }
      else if (clazz.equals(float.class) || clazz.equals(Float.class))
      {
         buffer += "float32";
      }
      else if (clazz.equals(double.class) || clazz.equals(Double.class))
      {
         buffer += "float64";
      }
      else if (clazz.equals(boolean.class) || clazz.equals(Boolean.class))
      {
         buffer += "bool";
      }
      else if (clazz.equals(char.class) || clazz.equals(Character.class))
      {
         buffer += "uint8";
      }
      else if (clazz.equals(String.class))
      {
         buffer += "string";
      }

      else if (clazz.equals(Quat4d.class))
      {
         buffer += "geometry_msgs/Quaternion";
      }
      else if (clazz.equals(Point3d.class) || (clazz.equals(Vector3d.class)) || clazz.equals(Vector3f.class))
      {
         buffer += "geometry_msgs/Vector3";
      }
      else
      {
         buffer += createNewRosMessage(clazz, overwriteSubMessages);
      }

      return buffer;
   }

// private void printAnnotation(String annotaionString, String printBuffer)
// {
//       String[] annotationString = annotaionString.split("\r?\n|\r");
//       for(String line : annotationString)
//       {
//               printBuffer += "# " + line + "\n";
//       }
// }
}
