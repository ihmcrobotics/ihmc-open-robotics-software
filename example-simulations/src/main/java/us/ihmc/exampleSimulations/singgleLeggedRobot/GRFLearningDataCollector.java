package us.ihmc.exampleSimulations.singgleLeggedRobot;
// Data saving for using in TensorFlow.

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

public class GRFLearningDataCollector
{
   // For data saving
   private double[] GRFzSave;

   private double[] qHipSave;
   private double[] tauHipSave;
   private double[] qKneeSave;
   private double[] tauKneeSave;
   private double[] qddZSave;
   private int dataIdx;
   private double fileIdx;
   private int numOfDataPerTrajectory;

   private SinggleLeggedRobot robot;

   
   
   public GRFLearningDataCollector(SinggleLeggedRobot robot, int numOfDataPerTrajectory)
   {
      this.robot = robot;
      this.numOfDataPerTrajectory = numOfDataPerTrajectory;
      initialize();
   }
   
   public void saveData()
   {
//      System.out.println("dataIdx : " + dataIdx);
      qHipSave[dataIdx] = robot.getHipAngularPosition();
      qKneeSave[dataIdx] = robot.getKneeAngularPosition();
      tauHipSave[dataIdx] = robot.getHipTau();
      tauKneeSave[dataIdx] = robot.getKneeTau();
      qddZSave[dataIdx] = robot.getHipSlideAcceleration();
      GRFzSave[dataIdx] = robot.getGRFz();
      dataIdx++;
   }

   private void initialize()
   {
      fileIdx = 0.0;
      dataIdx = 0;
      qHipSave = new double[numOfDataPerTrajectory];
      qKneeSave = new double[numOfDataPerTrajectory];
      tauHipSave = new double[numOfDataPerTrajectory];
      tauKneeSave = new double[numOfDataPerTrajectory];
      qddZSave = new double[numOfDataPerTrajectory];
      GRFzSave = new double[numOfDataPerTrajectory];
   }

   public void clearData()
   {
      System.out.println("data Clear");
      dataIdx = 0;
      qHipSave = new double[numOfDataPerTrajectory];
      qKneeSave = new double[numOfDataPerTrajectory];
      tauHipSave = new double[numOfDataPerTrajectory];
      tauKneeSave = new double[numOfDataPerTrajectory];
      qddZSave = new double[numOfDataPerTrajectory];
      GRFzSave = new double[numOfDataPerTrajectory];
   }

   public void saveFile()
   {
      System.out.println("data saving, sim Time:" + robot.getTime());
      String fileIdxString = Double.toString(fileIdx);
      String filePath = "C:/GRFdata_java/test" + fileIdxString + ".csv";
      File file = null;
      BufferedWriter bw = null;
      String NEWLINE = System.lineSeparator();
      String tempString;

      try
      {
         file = new File(filePath);
         bw = new BufferedWriter(new FileWriter(file));

         for (int i = 0; i < numOfDataPerTrajectory; i++)
         {
            tempString = Double.toString(qHipSave[i]);
            bw.write(tempString);
            bw.write(",");
            tempString = Double.toString(qKneeSave[i]);
            bw.write(tempString);
            bw.write(",");
            tempString = Double.toString(tauHipSave[i]);
            bw.write(tempString);
            bw.write(",");
            tempString = Double.toString(tauKneeSave[i]);
            bw.write(tempString);
            bw.write(",");
            tempString = Double.toString(qddZSave[i]);
            bw.write(tempString);
            bw.write(",");
            tempString = Double.toString(GRFzSave[i]);
            bw.write(tempString);

            bw.write(NEWLINE);
         }
         bw.flush();
         bw.close();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      fileIdx += 1.0;
   }
   
   public int getDataIdx()
   {
      return dataIdx;
   }
}
