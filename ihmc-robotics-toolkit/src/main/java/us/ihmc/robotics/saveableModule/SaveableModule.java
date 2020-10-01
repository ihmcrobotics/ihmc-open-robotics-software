package us.ihmc.robotics.saveableModule;

import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.io.*;
import java.util.*;

public class SaveableModule
{
   private final List<YoDouble> doubles = new ArrayList<>();
   private final List<YoInteger> integers = new ArrayList<>();
   private final List<YoBoolean> booleans = new ArrayList<>();

   private File file;

   public void setFileToSave(File file)
   {
      this.file = SaveableModuleTools.ensureFileExists(file);
   }

   public void setFileToSave(String filePath)
   {
      this.file = SaveableModuleTools.ensureFileExists(new File(filePath));
   }

   public void registerDoubleToSave(YoDouble dble)
   {
      doubles.add(dble);
   }

   public void registerIntegerToSave(YoInteger integer)
   {
      integers.add(integer);
   }

   public void registerBooleanToSave(YoBoolean yoBoolean)
   {
      booleans.add(yoBoolean);
   }

   public void registerYoFramePoint3DToSave(YoFramePoint3D framePoint3D)
   {
      registerDoubleToSave(framePoint3D.getYoX());
      registerDoubleToSave(framePoint3D.getYoY());
      registerDoubleToSave(framePoint3D.getYoZ());
   }

   public void registerYoFramePoint2DToSave(YoFramePoint2D framePoint2D)
   {
      registerDoubleToSave(framePoint2D.getYoX());
      registerDoubleToSave(framePoint2D.getYoY());
   }

   public void registerYoFrameQuaternionToSave(YoFrameQuaternion frameQuaternion)
   {
      registerDoubleToSave(frameQuaternion.getYoQs());
      registerDoubleToSave(frameQuaternion.getYoQx());
      registerDoubleToSave(frameQuaternion.getYoQy());
      registerDoubleToSave(frameQuaternion.getYoQz());
   }

   public void registerYoFramePose3DToSave(YoFramePose3D framePose3D)
   {
      registerYoFramePoint3DToSave(framePose3D.getPosition());
      registerYoFrameQuaternionToSave(framePose3D.getOrientation());
   }

   @Override
   public String toString()
   {
      String string = "";
      for (int i = 0; i < doubles.size(); i++)
      {
         YoDouble yoDouble = doubles.get(i);
         string += yoDouble.getFullNameString() + ": " + yoDouble.getValue() + ", ";
      }
      for (int i = 0; i < integers.size(); i++)
      {
         YoInteger yoInteger = integers.get(i);
         string += yoInteger.getFullNameString() + ": " + yoInteger.getValue() + ", ";
      }
      for (int i = 0; i < booleans.size(); i++)
      {
         YoBoolean yoBoolean = booleans.get(i);
         string += yoBoolean.getFullNameString() + ": " + yoBoolean.getValue() + ", ";
      }

      return string;
   }

   public void load(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      for (int i = 0; i < doubles.size(); i++)
         doubles.get(i).set(SaveableModuleTools.readNextDouble(scanner));
      for (int i = 0; i < integers.size(); i++)
         integers.get(i).set(SaveableModuleTools.readNextInt(scanner));
      for (int i = 0; i < booleans.size(); i++)
         booleans.get(i).set(SaveableModuleTools.readNextBoolean(scanner));
   }

   public void save(String propertyName)
   {
      if (file == null)
      {
         throw new IllegalArgumentException("File has not been set.");
      }

      Properties properties = new Properties()
      {
         private static final long serialVersionUID = -8814683165980261816L;

         @Override
         public synchronized Enumeration<Object> keys()
         {
            return Collections.enumeration(new TreeSet<Object>(super.keySet()));
         }
      };

      FileInputStream fileIn = null;
      FileOutputStream fileOut = null;

      try
      {
         if (file.exists() && file.isFile())
         {
            fileIn = new FileInputStream(file);
            properties.load(fileIn);
         }

         properties.setProperty(propertyName, toString());
         fileOut = new FileOutputStream(file);
         properties.store(fileOut, "");
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when saving property.");
      }
      finally
      {
         SaveableModuleTools.closeStreamSilently(fileIn);
         SaveableModuleTools.closeStreamSilently(fileOut);
      }
   }


}
