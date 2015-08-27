package us.ihmc.atlas;

import java.io.File;
import java.io.PrintWriter;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;

public class DRCRobotModelFileInvestigator
{
   public void writeModelFile(DRCRobotModel model)
   {
      try
      {
         String filename = "scsRobotModelOutputs/scsRobot_" + model;
         File file = new File(filename);

         PrintWriter printWriter = new PrintWriter(file);

         final SDFRobot robot = model.createSdfRobot(false);

         printWriter.println(robot);
         printWriter.close();
      }
      catch (Exception e)
      {
         System.err.println("Caught exception with model " + model);
         e.printStackTrace();
      }
   }

   public void writeAllModelFiles()
   {
      for (String st : AtlasRobotModelFactory.getAvailableRobotModels())
      {
         writeModelFile(AtlasRobotModelFactory.createDRCRobotModel(st, DRCRobotModel.RobotTarget.SCS, false));
      }
   }

   public static void main(String[] args)
   {
      DRCRobotModelFileInvestigator investigator = new DRCRobotModelFileInvestigator();
      investigator.writeAllModelFiles();
   }

}
