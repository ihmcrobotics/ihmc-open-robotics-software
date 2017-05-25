package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;

public class SolarPanelCleaningInfo
{
   private static SolarPanel solarPanel;
   private static SolarPanelPath cleaningPath;
   private static DegreesOfRedundancy cleaningPoseDegrees;
   private static CleaningPathType cleaningType;
   
   private static SolarPanelCleaningPose readyPose;
   
   public enum CleaningPathType
   {
      HORIZONAL, DIAGONAL
   }
   
   public enum DegreesOfRedundancy
   {
      ONE, THREE
   }
   
   public SolarPanelCleaningInfo()
   {
      
   }
   
   public static void setSolarPanel(SolarPanel solarPanel)
   {
      SolarPanelCleaningInfo.solarPanel = solarPanel;
   }
   
   public static SolarPanel getSolarPanel()
   {
      return solarPanel;
   }
   
   public static void setReadyPose(SolarPanelCleaningPose pose)
   {
      SolarPanelCleaningInfo.readyPose = pose;
   }
   
   public static SolarPanelCleaningPose getReadyPose()
   {
      return readyPose;
   }
      
   public static void setCleaningPathType(CleaningPathType type)
   {
      SolarPanelCleaningInfo.cleaningType = type;
   }
   
   public static CleaningPathType getCleaningType()
   {
      return cleaningType;
   }
   
   public static void setCleaningPath(SolarPanelPath path)
   {
      SolarPanelCleaningInfo.cleaningPath = path;
   }
   
   public static void setCleaningPath(CleaningPathType type)
   {
      switch(type)
      {
      
      }
      
      
      
    
//      cleaningPath = new SolarPanelPath(readyPose);
//      
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.1, -0.15, -Math.PI*0.3), 5.0);         
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.2, -0.15, -Math.PI*0.3), 2.5);
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.2, -0.15, -Math.PI*0.1), 5.0);
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.3, -0.15, -Math.PI*0.1), 2.5);
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.3, -0.15, -Math.PI*0.3), 5.0);
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.4, -0.15, -Math.PI*0.3), 2.5);
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.4, -0.15, -Math.PI*0.1), 5.0);
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.5, 0.5, -0.15, -Math.PI*0.1), 2.5);
//      cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, 0.1, 0.5, -0.15, -Math.PI*0.1), 5.0);
   }
   
   public static SolarPanelPath getCleaningPath()
   {
      return cleaningPath;
   }
   
   public static void setDegreesOfRedundancy(DegreesOfRedundancy degrees)
   {
      SolarPanelCleaningInfo.cleaningPoseDegrees = degrees;
   }
   
   public static DegreesOfRedundancy getDegreesOfRedundancy()
   {
      return cleaningPoseDegrees;
   }
   
   public static RRTNode getNode()
   {
      switch(cleaningPoseDegrees)
      {
         case ONE:
         {
            return new TimeDomain1DNode();             
         }
         case THREE:
         {
            return new TimeDomain3DNode();            
         }
      }
      
      return null;
   }

}
