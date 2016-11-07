package us.ihmc.atlas;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;

import com.martiansoftware.jsap.FlaggedOption;
import com.martiansoftware.jsap.JSAP;
import com.martiansoftware.jsap.JSAPResult;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;

public class AtlasStatisticsDisplay
{
   public static void main(String[] args)
   {
      // Flag to set robot model
      JSAP jsap = new JSAP();
      FlaggedOption robotModel = new FlaggedOption("robotModel").setLongFlag("model").setShortFlag('m').setRequired(true).setStringParser(JSAP.STRING_PARSER);
      robotModel.setHelp("Robot models: " + AtlasRobotModelFactory.robotModelsToString());

      DRCRobotModel model;
      try
      {
         jsap.registerParameter(robotModel);

         JSAPResult config = jsap.parse(args);

         if (config.success())
         {
            model = AtlasRobotModelFactory.createDRCRobotModel(config.getString("robotModel"), DRCRobotModel.RobotTarget.SCS, false);
         }
         else
         {
            System.out.println("Enter a robot model.");

            return;
         }
      }
      catch (Exception e)
      {
         System.out.println("Robot model not found");
         e.printStackTrace();

         return;
      }

      try
      {
    	  Class<?> clazz = Class.forName("us.ihmc.humanoidOperatorInterface.networking.StatisticsDisplay");
    	  Constructor<?> constructor = clazz.getDeclaredConstructor(FullHumanoidRobotModel.class);
    	  constructor.newInstance(model.createFullRobotModel());
      }
      catch (ClassNotFoundException e)
      {
        // TODO insert Dennis Nedry troll here
    	  e.printStackTrace();
      }
      catch (NoSuchMethodException | SecurityException | InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
    	  e.printStackTrace();
      }
   }
}
