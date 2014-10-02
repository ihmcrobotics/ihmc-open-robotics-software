package us.ihmc.communication.ros;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFileChooser;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.walking.HeadOrientationPacket;
import us.ihmc.communication.packets.walking.PauseCommand;

public class ROSMessageGenerator
{

   private String messageFolder;
   boolean overwriteSubMessages;

   public ROSMessageGenerator(String messageFolder, Boolean overwriteSubMessages)
   {
      this.messageFolder = messageFolder;
   }

   public static void main(String... args)
   {
      List<Class> toConvert = new ArrayList<Class>();
      toConvert.add(PauseCommand.class);
      toConvert.add(HeadOrientationPacket.class);
      toConvert.add(HandPosePacket.class);

      
      String destination = "us/ihmc/communication/msgs/";
      String messageFolder = ("generated/" + destination).replace("/", File.separator);
      File file = new File(messageFolder);
      if (!file.exists())
      {
         file.mkdirs();
      }
      
      System.out.println(messageFolder);
      System.out.println();
      ROSMessageGenerator messageGenerator = new ROSMessageGenerator(messageFolder, false);
      for (Class clazz : toConvert)
      {
         messageGenerator.createNewRosMessage(clazz, true);
      }
   }

   public String createNewRosMessage(Class clazz, boolean overwrite)
   {
      if (clazz == null)
      {
         return "";
      }
      String messageName = clazz.getSimpleName() + ".msg";
      File messageFile = new File(messageFolder + "\\" + messageName);

      if (overwrite || !messageFile.exists())
      {
         messageFile.delete();
         try
         {
            messageFile.createNewFile();
            System.out.println("Message Created: " + messageFile.getName());
            PrintStream fileStream = new PrintStream(messageFile);
            
            String outBuffer = "// " + messageName + System.lineSeparator() + System.lineSeparator();

            Field[] fields = clazz.getFields();
            for (Field field : fields)
            {
               outBuffer += printType(field.getType());
               outBuffer += " " + field.getName() + System.lineSeparator();
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

   private String printType(Class clazz)
   {
      String buffer = "";
      if (clazz == null)
      {
         return buffer;
      }

      if (clazz.isArray())
      {
         buffer += printType(clazz.getComponentType());
         buffer += "[]";
      }
      else if (clazz.equals(boolean.class))
      {
         buffer += "bool";
      }
      else if (clazz.equals(double.class))
      {
         buffer += "float64";
      }
      else if (clazz.equals(Quat4d.class))
      {
         buffer += "geometry_msgs/Quaternion";
      }
      else if (clazz.equals(Point3d.class))
      {
         buffer += "geometry_msgs/Vector3";
      }
      else if (clazz.isEnum())
      {
         Object[] enumList = clazz.getEnumConstants();
         for (int i = 0; i < enumList.length; i++)
         {
            buffer += "uint8";
            buffer += " " + enumList[i];
            buffer += " = " + i + System.lineSeparator();
         }
         buffer += "uint8";
      }
      else
      {
         buffer += createNewRosMessage(clazz, overwriteSubMessages);
      }
      return buffer;
   }
}



