package us.ihmc.tools.calibration;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Enumeration;
import java.util.Properties;
import java.util.StringTokenizer;

/**
 * CalibrationProperties
 *
 * Reads and sets Calibration Properties Files.
 *
 * If Neither the Default calibration file or the Current calibration file
 * exists, they will both be created on the first run with any new properties
 * that have been set
 *
 * If only the Default calibration file does not exist it will be created on
 * first run.
 *
 * If only the Current calibration file does not exist it will be created with
 * the values from the Default calibration file.
 *
 * Company: IHMC
 *
 * @author John Carff, Twan Koolen
 */
public class CalibrationProperties
{
   private final Properties properties;

   private final String defaultDirectory;
   private final String propertiesFile;

   /**
    * create a calibration file in the default locations
    */
   public CalibrationProperties()
   {
      this("./calibration", "currentCalibration.property");
   }

   /**
    * create a calibration file in the specified default and current locations
    */
   public CalibrationProperties(String directory, String currentPropertiesFile)
   {
      this.defaultDirectory = directory;
      this.propertiesFile = defaultDirectory + "/" + currentPropertiesFile;

      properties = new Properties();

      try
      {
         FileInputStream inStream = new FileInputStream(propertiesFile);
         properties.load(inStream);
         inStream.close();
      }
      catch (IOException e)
      {
         System.out.println("Properties file does not exist. No properties were set and a new file will be created when save is called.");
      }
   }

   /**
    * return the specified property's value as a Double
    */
   public double getDoubleProperty(String key)
   {
      String stringProperty = properties.getProperty(key);
      if (stringProperty == null)
      {
         double defaultValue = 0.0;
         System.out.println("Property " + key + " not found. Creating it and defaulting to " + defaultValue);
         properties.setProperty(key, Double.toString(defaultValue));
         return defaultValue;
      }
      else
      {
         double property = Double.valueOf(stringProperty);
         return property;
      }
   }

   /**
    * return the specified property's value as an Integer
    */
   public int getIntegerProperty(String key)
   {
      String stringProperty = properties.getProperty(key);
      if (stringProperty == null)
      {
         int defaultValue = 0;
         System.out.println("Property " + key + " not found. Creating it and defaulting to " + defaultValue);
         properties.setProperty(key, Integer.toString(defaultValue));
         return defaultValue;
      }
      else
      {
         int property = (int) expressionToValue(stringProperty);
         return property;
      }
   }

   /**
    * Sets the specified property(key) with the specified string value
    */
   public void setProperty(String key, String value)
   {
      properties.setProperty(key, value);
   }
   
   /**
    * Sets the specified property(key) with the specified double value
    */
   public void setProperty(String key, double value)
   {
      setProperty(key, Double.toString(value));
   }

   /**
    * Sets the specifed property(key) with the specified integer value
    */
   public void setProperty(String key, int value)
   {
      setProperty(key, Integer.toString(value));
   }

   /**
    * saves the property file to the location specified at creation.
    */
   public void save()
   {
      File dir = new File(defaultDirectory);
      if (!dir.exists())
      {
         System.out.println("CREATED DIRECTORY:" + dir.mkdir());
      }

      try
      {
         FileOutputStream outStream = new FileOutputStream(propertiesFile);
         properties.store(outStream, null);
         outStream.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   /**
    * Replaces expressions (e.g. 324 + 5 - 2) by their values (e.g. 327) and saves to disk
    */
   public void replaceExpressionsByValuesAndSave()
   {
      for (@SuppressWarnings("unchecked")
      Enumeration<String> propertyNameEnumeration = (Enumeration<String>) properties.propertyNames(); propertyNameEnumeration.hasMoreElements();)
      {
         String propertyName = propertyNameEnumeration.nextElement();
         String expression = properties.getProperty(propertyName);
         double value = expressionToValue(expression);
         setProperty(propertyName, value);
      }
      save();
   }

   /**
    * deals with simple addition and subtraction expressions.
    */
   private double expressionToValue(String expression)
   {
      boolean returnDelims = true; // delimiter characters are themselves considered to be tokens
      String plusToken = "+";
      String minusToken = "-";
      String operationTokens = plusToken + minusToken;
      StringTokenizer tokenizer = new StringTokenizer(expression, operationTokens, returnDelims);

      double ret = 0.0;
      int tokenCount = tokenizer.countTokens();
      int nextOperation = +1; // the next operation to perform; 0 signifies there is no unprocessed operation on the stack, +1 is +, -1 is -.
      for (int i = 0; i < tokenCount; i++)
      {
         String token = tokenizer.nextToken().trim();
         if (token.equals(plusToken) || token.equals(minusToken))
         {
            int operationValue = token.equals(plusToken) ? +1 : -1;
            if (nextOperation == 0)
            {
               nextOperation = operationValue;
            }
            else
            {
               nextOperation *= operationValue;
            }
         }
         else
         {
            double doubleValue = Double.valueOf(token); // will throw a NumberFormatException if the token is not correctly formatted
            ret += nextOperation * doubleValue;
            nextOperation = 0;
         }
      }
      
      return ret;
   }
}



/*
 * copied calibration file in case the files get deleted.
 *
 * #Mon Oct 19 15:48:48 CDT 2009
 *
 * // //AmplifierBoardConfiguration //
 * leftHipYawForceEncoderIndexPulseOffset=-8.0
 * leftHipYawPositionEncoderIndexPulseOffset=-701.0 //
 * leftHipRollForceEncoderIndexPulseOffset=0.0
 * leftHipRollPositionEncoderIndexPulseOffset=-717.0 //
 * leftHipPitchForceEncoderIndexPulseOffset=43.0
 * leftHipPitchPositionEncoderIndexPulseOffset=1328.0 //
 * leftKneeForceEncoderIndexPulseOffset=-30.0
 * leftKneePositionEncoderIndexPulseOffset=3284.0 //
 * leftAnkleOutForceEncoderIndexPulseOffset=-2.0
 * leftAnkleOutPositionEncoderIndexPulseOffset=-920.0 //
 * leftAnkleInForceEncoderIndexPulseOffset=-45.0
 * leftAnkleInPositionEncoderIndexPulseOffset=1071.0 //
 * rightHipYawForceEncoderIndexPulseOffset=0.0
 * rightHipYawPositionEncoderIndexPulseOffset=-685.0 //
 * rightHipRollForceEncoderIndexPulseOffset=0.0
 * rightHipRollPositionEncoderIndexPulseOffset=-893.0 //
 * rightHipPitchForceEncoderIndexPulseOffset=-66.0
 * rightHipPitchPositionEncoderIndexPulseOffset=-1052.0 //
 * rightKneeForceEncoderIndexPulseOffset=-7.0
 * rightKneePositionEncoderIndexPulseOffset=-3083.0 //
 * rightAnkleInForceEncoderIndexPulseOffset=-19.0
 * rightAnkleInPositionEncoderIndexPulseOffset=-983.0 //
 * rightAnkleOutForceEncoderIndexPulseOffset=58.0
 * rightAnkleOutPositionEncoderIndexPulseOffset=1059.0
 *
 *
 * // //YoboticsBipedRawSensorReader // // accelXVoltsAtLevel=2.605
 * accelYVoltsAtLevel=2.632 accelZVoltsAtLevel=2.597 // compassYVoltsOffset=2.5
 * compassXVoltsOffset=2.65 compassZVoltsOffset=2.5
 */



