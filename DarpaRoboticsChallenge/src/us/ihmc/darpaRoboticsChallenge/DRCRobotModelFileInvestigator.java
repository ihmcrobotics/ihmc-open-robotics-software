package us.ihmc.darpaRoboticsChallenge;

import java.io.File;
import java.io.PrintWriter;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

public class DRCRobotModelFileInvestigator
{
   public void writeModelFile(DRCRobotModel model)
   {
      try
      {
         String filename = "scsRobotModelOutputs/scsRobot_" + model;
         File file = new File(filename);

         PrintWriter printWriter = new PrintWriter(file);

         DRCRobotJointMap jointMap = model.getJointMap(false, false);
         JaxbSDFLoader robotLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
         final SDFRobot robot = robotLoader.createRobot(jointMap, false);

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
      for (String st : DRCRobotModelFactory.getAvailableRobotModels())
      {
         writeModelFile(DRCRobotModelFactory.CreateDRCRobotModel(st));
      }
   }

   public static void main(String[] args)
   {
      DRCRobotModelFileInvestigator investigator = new DRCRobotModelFileInvestigator();
      investigator.writeAllModelFiles();
   }

}
