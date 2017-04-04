package us.ihmc.manipulation.planning.manipulation.solarpanelmotion.tobedeleted;

import us.ihmc.manipulation.planning.manipulation.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;

public class SolarPanelStraightPathold extends SolarPanelAbstractPathold
{   
   private SolarPanelCleaningPose startPose;
   private SolarPanelCleaningPose endPose;
   
   public SolarPanelStraightPathold(SolarPanel solarPanel, double motionTime, SolarPanelCleaningPose startPose, SolarPanelCleaningPose endPose, int numberOfWayPoints)
   {
      super(solarPanel, motionTime, numberOfWayPoints);      
      this.startPose = startPose;
      this.endPose = endPose;
      initPoses();
   }
   
   private void initPoses()
   {
      poses.clear();
      for (int i =0;i<numberOfWayPoints;i++)
      {
         double wayU = startPose.getU() + (endPose.getU() - startPose.getU()) * i / (numberOfWayPoints-1);
         double wayV = startPose.getV() + (endPose.getV() - startPose.getV()) * i / (numberOfWayPoints-1);
         double wayW = startPose.getW() + (endPose.getW() - startPose.getW()) * i / (numberOfWayPoints-1);
         
         // temporary
         double wayYaw = startPose.getZRotation() + (endPose.getZRotation() - startPose.getZRotation()) * i / (numberOfWayPoints-1);
         
         SolarPanelCleaningPose wayPose = new SolarPanelCleaningPose(solarPanel, wayU, wayV, wayW, wayYaw);
         poses.add(wayPose);
      }
   }
   
   public SolarPanelCleaningPose getStartPose()
   {
      return startPose;
   }
   
   public SolarPanelCleaningPose getEndPose()
   {
      return endPose;
   }

   @Override
   public void getPiecewise()
   {
      
      
   }
   

   

}
