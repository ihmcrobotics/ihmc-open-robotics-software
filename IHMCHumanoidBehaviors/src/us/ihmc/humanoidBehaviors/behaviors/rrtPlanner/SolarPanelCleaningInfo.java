package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanel;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelCleaningPose;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;

public class SolarPanelCleaningInfo
{
   private static SolarPanel solarPanel;
   private static SolarPanelCleaningPose readyPose;
   private static SolarPanelPath cleaningPath;
   private static DegreesOfRedundancy cleaningPoseDegrees;
   private static CleaningPathType cleaningType;      
   
   private static double widthOfBezel = 0.03;
   private static double offsetAlongToZdirection = -0.055;
   private static double offsetReadyPose = -0.15;
   
   public enum CleaningPathType
   {
      HORIZONAL, DIAGONAL, HORIZONAL_FIXED
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
      cleaningType = type;
      switch(type)
      {
         case HORIZONAL:
         {
            PrintTools.info("HORIZONAL");
            double radiusOfTowel = 0.09;
            
            readyPose = new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, radiusOfTowel+widthOfBezel+0.1, offsetAlongToZdirection + offsetReadyPose, -Math.PI*0.00);          
            SolarPanelCleaningPose startPose = new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, radiusOfTowel+widthOfBezel, offsetAlongToZdirection, -Math.PI*0.2);
            cleaningPath = new SolarPanelPath(startPose);
            
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel+widthOfBezel, radiusOfTowel+widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 6.0);
            
            int numberOfHorizonalLine = (int) ((solarPanel.getSizeV()-radiusOfTowel*2-widthOfBezel*2)/radiusOfTowel) + 2;
            
            for(int i=2;i<(numberOfHorizonalLine+1);i++)
            {               
               if(i != numberOfHorizonalLine)
               {
                  if(i%2 == 0)
                  {
                     cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel+widthOfBezel, radiusOfTowel*i+widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 3.0);
                     cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, radiusOfTowel*i+widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 6.0);                  
                  }
                  else
                  {
                     cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, radiusOfTowel*i+widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 3.0);
                     cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel+widthOfBezel, radiusOfTowel*i+widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 6.0);
                  }   
               }
               else
               {
                  if(i%2 == 0)
                  {
                     cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel+widthOfBezel, solarPanel.getSizeV()-radiusOfTowel-widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 6.0);
                     cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, solarPanel.getSizeV()-radiusOfTowel-widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 6.0);
                  }
                  else
                  {
                     cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, solarPanel.getSizeV()-radiusOfTowel-widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 6.0);
                     cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel+widthOfBezel, solarPanel.getSizeV()-radiusOfTowel-widthOfBezel, offsetAlongToZdirection, -Math.PI*0.15), 6.0);
                  }      
               }                  
            }
                        
            
            
            break;
         }
         case DIAGONAL:
         {
            PrintTools.info("DIAGONAL");
            double nomalizedTimeRatio = 15.0;
            double radiusOfTowel = 0.09;
            
            readyPose = new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, radiusOfTowel+widthOfBezel+0.1, offsetAlongToZdirection + offsetReadyPose, -Math.PI*0.00);   
            
            SolarPanelCleaningPose startPose = new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, radiusOfTowel+widthOfBezel, offsetAlongToZdirection, -Math.PI*0.2);
            cleaningPath = new SolarPanelPath(startPose);
            
            double totalBezel = radiusOfTowel + widthOfBezel;
            double oneStepU =  (solarPanel.getSizeU() - totalBezel*2)/4;
            double oneStepV =  (solarPanel.getSizeV() - totalBezel*2)/4;
            
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 1) , totalBezel + oneStepV * 0, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 0) , totalBezel + oneStepV * 1, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 2) , totalBezel + oneStepV * 0, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 0) , totalBezel + oneStepV * 2, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 3) , totalBezel + oneStepV * 0, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 0) , totalBezel + oneStepV * 3, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 4) , totalBezel + oneStepV * 0, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 0) , totalBezel + oneStepV * 4, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio); // 8
            
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 4) , totalBezel + oneStepV * 1, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 1) , totalBezel + oneStepV * 4, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 4) , totalBezel + oneStepV * 2, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 2) , totalBezel + oneStepV * 4, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 4) , totalBezel + oneStepV * 3, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 3) , totalBezel + oneStepV * 4, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - (totalBezel + oneStepU * 4) , totalBezel + oneStepV * 4, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio); // 15
            
            break;
         }
         
         case HORIZONAL_FIXED:
         {
            PrintTools.info("HORIZONAL_FIXED");
            double nomalizedTimeRatio = 15.0;
            double radiusOfTowel = 0.09;
            
            double totalBezel = radiusOfTowel + widthOfBezel;
            double oneStepV =  (solarPanel.getSizeV() - totalBezel*2)/4;
            
            readyPose = new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - radiusOfTowel - widthOfBezel, radiusOfTowel + widthOfBezel + 0.1, offsetAlongToZdirection + offsetReadyPose, -Math.PI*0.00);   
            
            SolarPanelCleaningPose startPose = new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - totalBezel, totalBezel, offsetAlongToZdirection, -Math.PI*0.2);
            cleaningPath = new SolarPanelPath(startPose);
            
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, totalBezel , totalBezel + oneStepV * 0, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, totalBezel , totalBezel + oneStepV * 1, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - totalBezel , totalBezel + oneStepV * 1, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - totalBezel , totalBezel + oneStepV * 2, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, totalBezel , totalBezel + oneStepV * 2, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, totalBezel , totalBezel + oneStepV * 3, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - totalBezel , totalBezel + oneStepV * 3, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU() - totalBezel , totalBezel + oneStepV * 4, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            cleaningPath.addCleaningPoseWithNomalizedTime(new SolarPanelCleaningPose(solarPanel, totalBezel , totalBezel + oneStepV * 4, offsetAlongToZdirection, -Math.PI*0.2), nomalizedTimeRatio);
            
            
            
            break;
         }
         
      }
      
      
      
    
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
