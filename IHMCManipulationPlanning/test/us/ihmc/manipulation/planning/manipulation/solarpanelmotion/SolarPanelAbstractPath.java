package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

public abstract class SolarPanelAbstractPath
{
   protected SolarPanel solarPanel;   
   protected double motionTime;   
   protected int numberOfWayPoints;   
   protected ArrayList<SolarPanelCleaningPose> poses = new ArrayList<SolarPanelCleaningPose>();
   protected ArrayList<SolarPanelCleaningPose> piecewisePoses = new ArrayList<SolarPanelCleaningPose>();
   protected int numberOfPiecewise;
   
   public SolarPanelAbstractPath(SolarPanel solarPanel, double motionTime, int numberOfWayPoints)
   {
      this.solarPanel = solarPanel;
      this.motionTime = motionTime;
      this.numberOfWayPoints = numberOfWayPoints;
      this.numberOfPiecewise = 10;
   }
   
   public int getNumberOfWayPoints()
   {
      return numberOfWayPoints;
   }
   
   public double getMotionTime()
   {
      return motionTime;
   }
   
   public ArrayList<SolarPanelCleaningPose> getPoses()
   {
      return poses;
   }
   
   public abstract void getPiecewise();
   
   public boolean isValidPath()
   {
      for(int i =0;i<piecewisePoses.size();i++)
      {
         if(piecewisePoses.get(i).isValid() == false)
         {
            return false;
         }
      }
      
      return true;
   }
}
