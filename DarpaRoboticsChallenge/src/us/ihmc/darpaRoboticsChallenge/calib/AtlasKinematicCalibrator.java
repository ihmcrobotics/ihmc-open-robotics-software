package us.ihmc.darpaRoboticsChallenge.calib;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.yobotics.simulationconstructionset.Joint;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;

import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;

public class AtlasKinematicCalibrator
{
   final SDFRobot robot;
   final ArrayList<OneDegreeOfFreedomJoint> joints=new ArrayList<>();
   final ArrayList<Map<String,Double>> q = new ArrayList<>();
   final ArrayList<Map<String,Double>> qout = new ArrayList<>();

   public AtlasKinematicCalibrator()
   {
      //load robot
      DRCRobotJointMap jointMap = new DRCRobotJointMap(DRCRobotModel.ATLAS_NO_HANDS_ADDED_MASS, false);
      JaxbSDFLoader robotLoader = DRCRobotSDFLoader.loadDRCRobot(jointMap);
      robot= robotLoader.createRobot(jointMap, false);
      robot.getAllOneDegreeOfFreedomJoints(joints);

   }
   
   public void loadJointAnglesFromFile()
   {
      String calib_file = "/home/unknownid/workspace/DarpaRoboticsChallenge/data/coupledWristLog_20131204";
      BufferedReader reader=null;
      try
      {
         reader = new BufferedReader(new FileReader(calib_file));
      }
      catch (FileNotFoundException e1)
      {
         System.out.println("Cannot load calibration file " + calib_file);
         e1.printStackTrace();
      }
      
      String line;
      final int numJoints = 28;
      System.out.println("total joints should be " + numJoints);
      try
      {
         while((line=reader.readLine())!=null)
         {
            if(line.matches("^entry.*")){
               Map<String, Double> q_ = new HashMap<>();
               Map<String, Double> qout_ = new HashMap<>();
               
               for(int i=0;i<numJoints;i++)
               {
                  line =reader.readLine();
                  if(line!=null)
                  {
                     String[] items= line.split("\\s");
                     q_.put(items[0], new Double(items[1]));
                     qout_.put(items[0], new Double(items[2]));
                  }
                  else
                  {
                     System.out.println("Ill-formed data entry");
                     break;
                  }
                  
               }

               if(q_.size()==numJoints)
                  q.add(q_);
               if(qout_.size()==numJoints)
                  qout.add(qout_);

            }
         }
      }
      catch (IOException e1)
      {
         System.err.println("File reading error");
         e1.printStackTrace();
      }
      System.out.println("total entry loaded q/qout " + q.size() + "/" + qout.size());
   }
   
   public void setRobotModel(int timeIndex)
   {
      Map<String, Double> qmap = q.get(timeIndex);
      for(int i=0;i<joints.size();i++)
      {
         OneDegreeOfFreedomJoint joint =joints.get(i);
         if (qmap.containsKey(joint.getName()))
         {
            joint.setQ(qmap.get(joint.getName()));
         }
         else if (joint.getName().equals("neck_ry"))
         {
            //special treatment 
            joint.setQ(qmap.get("neck_ay"));
         }
         else
         {
            System.out.println("model contain joints not in data "+joint.getName());
            joint.setQ(0);
         }
      }

   }
   
   
   
   private void displayRobot()
   {
      //implement a show robot thingy here.
   }   
   
   public static void main(String[] arg)
   {
      AtlasKinematicCalibrator calib = new AtlasKinematicCalibrator();
      calib.loadJointAnglesFromFile();
      calib.setRobotModel(100);
      calib.displayRobot();
   }


}
