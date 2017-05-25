package us.ihmc.manipulation.planning.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;

public class SolarPanelPath
{
   private ArrayList<SolarPanelCleaningPose> wayPoses = new ArrayList<SolarPanelCleaningPose>();
   private ArrayList<Double> arrivalTime = new ArrayList<Double>();
   private ArrayList<SolarPanelLinearPath> linearPath = new ArrayList<SolarPanelLinearPath>();

   public SolarPanelPath(SolarPanelCleaningPose startPose)
   {
      wayPoses.add(startPose);
      arrivalTime.add(0.0);
   }
   
   public SolarPanelCleaningPose getStartPose()
   {
      return wayPoses.get(0);
   }
   
   public SolarPanel getSolarPanel()
   {
      return wayPoses.get(0).getSolarPanel();
   }
   
   public int getNumerOfLinearPath()
   {
      return linearPath.size();
   }
   
   public ArrayList<Double> getArrivalTime()
   {
      return arrivalTime;
   }

   public ArrayList<SolarPanelLinearPath> getLinearPath()
   {
      return linearPath;
   }

   public void addCleaningPose(SolarPanelCleaningPose cleaningPose, double timeToGo)
   {
      SolarPanelCleaningPose newStartPose = wayPoses.get(wayPoses.size()-1);
      double startTime = arrivalTime.get(arrivalTime.size()-1);
      double endTime = startTime + timeToGo;

      SolarPanelLinearPath linearPath = new SolarPanelLinearPath(newStartPose, cleaningPose, startTime, endTime);
      this.linearPath.add(linearPath);

      this.wayPoses.add(cleaningPose);
      this.arrivalTime.add(endTime);
   }

   public SolarPanelCleaningPose getCleaningPose(double time)
   {
      int indexOfLinearPath = 0;

      if(arrivalTime.size() > 2)
      {
         for(int i =0;i<arrivalTime.size()-1;i++)
         {
            if(arrivalTime.get(i) < time && time <= arrivalTime.get(i+1))
            {
               indexOfLinearPath = i;
            }
         }
      }
      else
      {
         indexOfLinearPath = 0;
      }

      SolarPanelCleaningPose cleaningPose = linearPath.get(indexOfLinearPath).getInterpolatedCleaningPose(time);

      return cleaningPose;
   }

   public void reArrangementArrivalTime()
   {
      double totalPathTime = arrivalTime.get(arrivalTime.size()-1);
      
      arrivalTime.clear();
      arrivalTime.add(0.0);      
      
      double totalPathLength = 0;
      
      for(int i=0;i<getNumerOfLinearPath();i++)
      {
         double subPathLength = linearPath.get(i).getPathLength();
         totalPathLength = totalPathLength + subPathLength;
      }
      
      for(int i=0;i<getNumerOfLinearPath();i++)
      {
         double subPathLength = linearPath.get(i).getPathLength();
         double subPathTime = subPathLength/totalPathLength * totalPathTime;
         double previousArrivalTime = arrivalTime.get(arrivalTime.size()-1);
         double newArrivalTime = previousArrivalTime + subPathTime;
         arrivalTime.add(newArrivalTime);
         PrintTools.info(""+i+" "+newArrivalTime);
      }
      
      for(int i=0;i<getNumerOfLinearPath()-1;i++)
      {
         linearPath.get(i).setMotionStartTime(arrivalTime.get(i));
         linearPath.get(i).setMotionEndTime(arrivalTime.get(i+1));
      }
   }
   
}
