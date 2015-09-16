package us.ihmc.atlas;

import java.io.IOException;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.ROSAPISimulator;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;

public class AtlasDemo01ROSAPISimulator extends ROSAPISimulator
{
   private static final String ROBOT_NAME = "atlas";
   private static final String DEFAULT_ROBOT_MODEL = "ATLAS_UNPLUGGED_V5_NO_HANDS";
   
   public AtlasDemo01ROSAPISimulator(DRCRobotModel robotModel, DRCStartingLocation startingLocation, String nameSpace, String tfPrefix,
         boolean runAutomaticDiagnosticRoutine, boolean disableViz) throws IOException
   {
      super(robotModel, startingLocation, nameSpace, tfPrefix, runAutomaticDiagnosticRoutine, disableViz);
   }

   @Override
   protected CommonAvatarEnvironmentInterface createEnvironment()
   {
      return new DRCDemo01NavigationEnvironment();
   }
   
   public static void main(String[] args) throws JSAPException, IOException
   {
      Options opt = parseArguments(args);
      
      DRCRobotModel robotModel;
      try
      {
         if (opt.robotModel.equals(DEFAULT_STRING))
         {
            robotModel = AtlasRobotModelFactory.createDRCRobotModel(DEFAULT_ROBOT_MODEL, DRCRobotModel.RobotTarget.SCS, false);
         }
         else
         {
            robotModel = AtlasRobotModelFactory.createDRCRobotModel(opt.robotModel, DRCRobotModel.RobotTarget.SCS, false);
         }
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect robot model " + opt.robotModel);
         System.out.println("Robot models: " + AtlasRobotModelFactory.robotModelsToString());
         return;
      }
      
      DRCStartingLocation startingLocation;
      try
      {
         startingLocation = DRCObstacleCourseStartingLocation.valueOf(opt.startingLocation);
      }
      catch (IllegalArgumentException e)
      {
         System.err.println("Incorrect starting location " + opt.startingLocation);
         System.out.println("Starting locations: " + DRCObstacleCourseStartingLocation.optionsToString());
         return;
      }
      
      String nameSpace = opt.nameSpace + "/" + ROBOT_NAME;
      new AtlasDemo01ROSAPISimulator(robotModel, startingLocation, nameSpace, opt.tfPrefix, opt.runAutomaticDiagnosticRoutine, opt.disableViz);
   }
}
