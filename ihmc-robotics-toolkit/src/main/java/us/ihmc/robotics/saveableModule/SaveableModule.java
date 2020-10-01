package us.ihmc.robotics.saveableModule;

import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Scanner;

public class SaveableModule
{
   private final List<YoDouble> doubles = new ArrayList<>();
   private final List<YoInteger> integers = new ArrayList<>();
   private final List<YoBoolean> booleans = new ArrayList<>();

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
         doubles.get(i).set(readNextDouble(scanner));
      for (int i = 0; i < integers.size(); i++)
         integers.get(i).set(readNextInt(scanner));
      for (int i = 0; i < booleans.size(); i++)
         booleans.get(i).set(readNextBoolean(scanner));
   }

   private static double readNextDouble(Scanner scanner)
   {
      return readNextDouble(scanner, Double.NaN);
   }

   private static double readNextDouble(Scanner scanner, double defaultValue)
   {
      try
      {
         while (!scanner.hasNextDouble())
            scanner.next();
         return scanner.nextDouble();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }

   public static int readNextInt(Scanner scanner)
   {
      return readNextInt(scanner, -1);
   }

   public static int readNextInt(Scanner scanner, int defaultValue)
   {
      try
      {
         while (!scanner.hasNextInt())
            scanner.next();
         return scanner.nextInt();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }

   public static boolean readNextBoolean(Scanner scanner)
   {
      return readNextBoolean(scanner, false);
   }

   public static boolean readNextBoolean(Scanner scanner, boolean defaultValue)
   {
      try
      {
         while (!scanner.hasNextBoolean())
            scanner.next();
         return scanner.nextBoolean();
      }
      catch (NoSuchElementException e)
      {
         return defaultValue;
      }
   }
}
