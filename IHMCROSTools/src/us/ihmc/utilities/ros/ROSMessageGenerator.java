package us.ihmc.utilities.ros;



import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.communication.packetAnnotations.FieldDocumentation;
import us.ihmc.communication.packetAnnotations.IgnoreField;
import us.ihmc.communication.packets.DocumentedPacket;
import us.ihmc.utilities.DocumentedEnum;
import us.ihmc.utilities.ros.msgToPacket.IHMCRosApiMessageMap;

public class ROSMessageGenerator
{
   private static String messageFolder = ("../ROSJavaBootstrap/ROSMessagesAndServices/ihmc_msgs/msg/").replace("/", File.separator);
   boolean overwriteSubMessages;

   public ROSMessageGenerator(Boolean overwriteSubMessages)
   {
      this.overwriteSubMessages = overwriteSubMessages;
   }

   public static void main(String... args) throws Exception
   {
      generate();
   }
   
   public static void generate() throws Exception
   {
      ROSMessageGenerator messageGenerator = new ROSMessageGenerator(true);
      for (Class clazz : IHMCRosApiMessageMap.PACKET_LIST)
      {
         messageGenerator.createNewRosMessage(clazz, true);
      }
   }

   public String createNewRosMessage(Class clazz, boolean overwrite) throws Exception
   {
      if (clazz == null)
      {
         return "";
      }
      
      if(!DocumentedPacket.class.isAssignableFrom(clazz))
         System.err.println(clazz.getSimpleName() + " does not extend DocumentedPacket");

      File file = new File(messageFolder);
      if (!file.exists())
      {
         file.mkdirs();
      }

      String messageName = clazz.getSimpleName() + "Message";
      File messageFile = new File((messageFolder + File.separator + messageName + ".msg"));

      if (overwrite ||!messageFile.exists())
      {
         messageFile.delete();

         try
         {
            messageFile.createNewFile();
            System.out.println("Message Created: " + messageFile.getName());
            PrintStream fileStream = new PrintStream(messageFile);

            String outBuffer = "## " + messageName + System.lineSeparator();
            ClassDocumentation annotation = (ClassDocumentation) clazz.getAnnotation(ClassDocumentation.class);
            if (annotation != null)
            {
            	String[] annotationString = annotation.documentation().split("\r?\n|\r");
         	   for(String line : annotationString)
         	   {
         		   outBuffer += "# " + line + System.lineSeparator();
         	   }
            }
            else
            {
               outBuffer += "# No Documentation Annotation Found" + System.lineSeparator();
            }

            outBuffer += System.lineSeparator();

            Field[] fields = clazz.getFields();
            for (Field field : fields)
            {
               if(isConstant(field) || isIgnoredField(field))
               {
                  continue;
               }

               outBuffer += printType(field);

               String formattedFieldName = camelCaseToLowerCaseWithUnderscores(field.getName());
               outBuffer += " " + formattedFieldName + System.lineSeparator() + System.lineSeparator();
            }

            fileStream.println(outBuffer);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      return messageName;
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
      return field.isAnnotationPresent(IgnoreField.class);
   }

   private boolean isConstant(Field field)
   {
      int modifier = field.getModifiers();

      return Modifier.isFinal(modifier) && Modifier.isPublic(modifier) && Modifier.isStatic(modifier);
   }

   private String printType(Field field) throws Exception
   {
	   String buffer = "";
	   FieldDocumentation fieldAnnotation = field.getAnnotation(FieldDocumentation.class);
	   if(fieldAnnotation != null)
	   {
		   String[] annotationString = fieldAnnotation.documentation().split("\r?\n|\r");
		   for(String line : annotationString)
		   {
			   buffer += "# " + line + System.lineSeparator();
		   }
	   }
	   
      if (List.class.isAssignableFrom(field.getType()))
      {
         Class genericType = ((Class) ((ParameterizedType) field.getGenericType()).getActualTypeArguments()[0]);
         buffer += printType(genericType, field.getName());
         buffer += "[]";
      }
      else
      {
         buffer += printType(field.getType(), field.getName());
      }

      return buffer;
   }

   @SuppressWarnings("unchecked")
   private String printType(Class clazz, String varName) throws Exception
   {
      String buffer = "";
      if (clazz == null)
      {
         return buffer;
      }

      if (clazz.isArray())
      {
         buffer += printType(clazz.getComponentType(), varName );
         buffer += "[]";
      }
      else if (clazz.isEnum())
      {
         Object[] enumList = clazz.getEnumConstants();
         buffer += "# Options for " + varName + System.lineSeparator();

         for (int i = 0; i < enumList.length; i++)
         {
            buffer += "# " + enumList[i];
            buffer += " = " + i;
            if (DocumentedEnum.class.isAssignableFrom(clazz))
            {
               DocumentedEnum<Object> documentedEnum = (DocumentedEnum<Object>) Enum.valueOf(clazz, enumList[i].toString());
               String documentation = documentedEnum.getDocumentation(enumList[i]);
               buffer += " - " + documentation;
            }
            else
            {
               System.err.println(clazz.getSimpleName() + " is not a DocumentedEnum!");
            }
            buffer += System.lineSeparator();
         }

         buffer += "uint8";
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
      else if (clazz.equals(Point3d.class) || (clazz.equals(Vector3d.class)))
      {
         buffer += "geometry_msgs/Vector3";
      }
      else
      {
         buffer += createNewRosMessage(clazz, overwriteSubMessages);
      }

      return buffer;
   }
   
//   private void printAnnotation(String annotaionString, String printBuffer)
//   {
//	   String[] annotationString = annotaionString.split("\r?\n|\r");
//	   for(String line : annotationString)
//	   {
//		   printBuffer += "# " + line + System.lineSeparator();
//	   }
//   }
}
