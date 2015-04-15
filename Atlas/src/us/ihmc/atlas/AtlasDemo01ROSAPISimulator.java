package us.ihmc.atlas;

import java.io.IOException;

import us.ihmc.darpaRoboticsChallenge.DRCObstacleCourseStartingLocation;
import us.ihmc.darpaRoboticsChallenge.DRCStartingLocation;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;

import com.martiansoftware.jsap.JSAPException;

public class AtlasDemo01ROSAPISimulator extends AtlasROSAPISimulator
{
   private static final String ROBOT_NAME = "atlas";
   
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
         robotModel = AtlasRobotModelFactory.createDRCRobotModel(opt.robotModel, AtlasRobotModel.AtlasTarget.SIM, false);
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
