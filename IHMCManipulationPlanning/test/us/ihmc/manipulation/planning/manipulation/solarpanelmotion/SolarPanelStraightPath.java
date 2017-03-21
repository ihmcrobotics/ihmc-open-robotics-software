package us.ihmc.manipulation.planning.manipulation.solarpanelmotion;

import java.util.ArrayList;

public class SolarPanelStraightPath
{
   private SolarPanel solarPanel;
   private SolarPanelCleaningPose startPose;
   private SolarPanelCleaningPose endPose;
   private int numberOfWayPoints;
   private ArrayList<SolarPanelCleaningPose> poses = new ArrayList<SolarPanelCleaningPose>();
   
   public SolarPanelStraightPath(SolarPanel solarPanel, SolarPanelCleaningPose startPose, SolarPanelCleaningPose endPose, int numberOfWayPoints)
   {
      this.solarPanel = solarPanel;
      this.startPose = startPose;
      this.endPose = endPose;
      this.numberOfWayPoints = numberOfWayPoints;    
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
   
   public ArrayList<SolarPanelCleaningPose> getPoses()
   {
      return poses;
   }
}
