package us.ihmc.atlas.highLevelHumanoidControl.highLevelStates;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.ros.AtlasOrderedJointMap;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandPrepSetpoints;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.HashMap;
import java.util.Properties;

public class NewAtlasStandPrepSetpoints implements StandPrepSetpoints
{
   private static final String defaultSetPointsFile = "AtlasStandPrepSetPoints.ini";
   //private static TDoubleList setPoints = new TDoubleArrayList(AtlasOrderedJointMap.numberOfJoints);
   private static HashMap<String, double[]> setPoints = new HashMap<>();

   static
   {
      useDefaultAngles();
      File file = new File(System.getProperty("standPrepSetPointsFile", defaultSetPointsFile));
      System.out.println("Attempting to load Stand Prep joint set points from " + file.getAbsolutePath());

      if(file.exists() && file.isFile())
      {
         loadCustomSetPoints(file);
      }
      else
      {
         System.err.println("Specified stand prep .ini file does not exist, using default values.");
      }
   }

   private static void loadCustomSetPoints(File file)
   {
      Properties standPrepProperties = new Properties();
      FileInputStream fileInputStream = null;
      try
      {
         fileInputStream = new FileInputStream(file);
      }
      catch (FileNotFoundException e)
      {
         System.err.println("Specified stand prep .ini file does not exist, using default values.");
      }

      if(fileInputStream != null)
      {
         try
         {
            standPrepProperties.load(fileInputStream);

            for (int jointIndex = 0; jointIndex < AtlasOrderedJointMap.jointNames.length; jointIndex++)
            {
               String propertyKey = AtlasOrderedJointMap.jointNames[jointIndex];
               if(standPrepProperties.containsKey(propertyKey))
               {
                  try
                  {
                     double value = Double.parseDouble(standPrepProperties.getProperty(propertyKey));
                     System.out.println("Found custom setpoint for " + propertyKey + ", setting to " + value);
                     set(jointIndex, value);
                  }
                  catch (NumberFormatException e)
                  {
                     System.err.println("Malformed property line for " + propertyKey + "set point, using default value");
                  }
               }
            }
         }
         catch (IOException e)
         {
            System.err.println("Problem loading the set points from the .ini file, using the default values");
         }

         try
         {
            fileInputStream.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   private static void useDefaultAngles()
   {                                      
      if (AtlasRobotModel.BATTERY_MASS_SIMULATOR_IN_ROBOT)
      {
         set(AtlasOrderedJointMap.back_bky, 0.13);
         set(AtlasOrderedJointMap.l_leg_aky, -0.77);
         set(AtlasOrderedJointMap.r_leg_aky, -0.77);
         set(AtlasOrderedJointMap.l_leg_kny, 1.75);
         set(AtlasOrderedJointMap.r_leg_kny, 1.75);
      }
      else
      {
         set(AtlasOrderedJointMap.back_bky, 0.0);
         set(AtlasOrderedJointMap.l_leg_aky, -0.75);
         set(AtlasOrderedJointMap.r_leg_aky, -0.75);
         set(AtlasOrderedJointMap.l_leg_kny, 1.65);
         set(AtlasOrderedJointMap.r_leg_kny, 1.65);
      }

      set(AtlasOrderedJointMap.back_bkz   , 0.0);
      set(AtlasOrderedJointMap.back_bkx   , 0.0);
      set(AtlasOrderedJointMap.neck_ry    , 0.0);
      set(AtlasOrderedJointMap.l_leg_hpz  , 0.0);
      set(AtlasOrderedJointMap.l_leg_hpx  , 0.09);
      set(AtlasOrderedJointMap.l_leg_hpy  , -0.933);
      set(AtlasOrderedJointMap.l_leg_akx  , -0.09);
      set(AtlasOrderedJointMap.r_leg_hpz  , 0.0);
      set(AtlasOrderedJointMap.r_leg_hpx  , -0.09);
      set(AtlasOrderedJointMap.r_leg_hpy  , -0.933);
      set(AtlasOrderedJointMap.r_leg_akx  , 0.09);

      set(AtlasOrderedJointMap.l_arm_shz  , 0.0);
      set(AtlasOrderedJointMap.l_arm_shx  , -1.3);
      set(AtlasOrderedJointMap.l_arm_ely  , 2.0);
      set(AtlasOrderedJointMap.l_arm_elx  , 0.5);
      set(AtlasOrderedJointMap.l_arm_wry  , 0.01);
      set(AtlasOrderedJointMap.l_arm_wrx  , 0.0);
      set(AtlasOrderedJointMap.l_arm_wry2  , 0.0);
      set(AtlasOrderedJointMap.r_arm_shz  , 0.0);
      set(AtlasOrderedJointMap.r_arm_shx  , 1.3);
      set(AtlasOrderedJointMap.r_arm_ely  , 2.0);
      set(AtlasOrderedJointMap.r_arm_elx  , -0.5);
      set(AtlasOrderedJointMap.r_arm_wry  , 0.01);
      set(AtlasOrderedJointMap.r_arm_wrx  , 0.0);
      set(AtlasOrderedJointMap.r_arm_wry2  , 0.0);
   }
   
   private static void set(int jointIndex, double value)
   {
      String jointName = AtlasOrderedJointMap.jointNames[jointIndex];

      if(setPoints.containsKey(jointName))
      {
         setPoints.remove(jointName);
      }
      setPoints.put(jointName, new double[] { value } );
   }

   public double get(int jointIndex)
   {
      String jointName = AtlasOrderedJointMap.jointNames[jointIndex];
      return setPoints.get(jointName)[0];
   }

   public double get(String jointName)
   {
      return setPoints.get(jointName)[0];
   }
}
