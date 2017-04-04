package us.ihmc.manipulation.planning.manipulation.solarpanelmotion.tobedeleted;

import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;

import java.util.ArrayList;

public abstract class SolarPanelAbstractPathold
{
   protected SolarPanel solarPanel;
   protected double motionTime;   
   protected int numberOfWayPoints;   
   protected ArrayList<SolarPanelCleaningPose> poses = new ArrayList<SolarPanelCleaningPose>();
   protected ArrayList<SolarPanelCleaningPose> piecewisePoses = new ArrayList<SolarPanelCleaningPose>();
   protected int numberOfPiecewise;
   
   public SolarPanelAbstractPathold(SolarPanel solarPanel, double motionTime, int numberOfWayPoints)
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

      }
      
      return true;
   }
}
