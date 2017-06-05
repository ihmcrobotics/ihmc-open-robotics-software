package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

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
   
   private static double widthOfBezel = 0.05;
   private static double offsetAlongToZdirection = -0.04;
   private static double offsetReadyPose = -0.15;
   
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
         case HORIZONAL:
         {
            double radiusOfTowel = 0.1;
            
            readyPose = new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel-widthOfBezel, radiusOfTowel+widthOfBezel, offsetAlongToZdirection + offsetReadyPose, -Math.PI*0.00);          
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
                        
//            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel, solarPanel.getSizeY()-radiusOfTowel, -0.15, -Math.PI*0.1), 2.0);
//            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeX()-radiusOfTowel, solarPanel.getSizeY()-radiusOfTowel, -0.15, -Math.PI*0.1), 7.0);
//            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeX()-radiusOfTowel, radiusOfTowel, -0.15, -Math.PI*0.1), 7.0);
//            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel, radiusOfTowel, -0.15, -Math.PI*0.1), 7.0);
//            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel, solarPanel.getSizeY()-radiusOfTowel, -0.15, -Math.PI*0.1), 7.0);
            
            break;
         }
         case DIAGONAL:
         {
            readyPose = new SolarPanelCleaningPose(solarPanel, 0.5, 0.1, -0.15, -Math.PI*0.1);
            
            cleaningPath = new SolarPanelPath(readyPose);
            
            double radiusOfTowel = 0.08;
            
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel, radiusOfTowel*1, -0.15, -Math.PI*0.3), 5.0);         
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel, radiusOfTowel*2, -0.15, -Math.PI*0.3), 2.5);
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel, radiusOfTowel*2, -0.15, -Math.PI*0.1), 5.0);
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel, radiusOfTowel*3, -0.15, -Math.PI*0.1), 2.5);
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel, radiusOfTowel*3, -0.15, -Math.PI*0.3), 5.0);
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel, radiusOfTowel*4, -0.15, -Math.PI*0.3), 2.5);
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel, radiusOfTowel*4, -0.15, -Math.PI*0.1), 5.0);
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, solarPanel.getSizeU()-radiusOfTowel, radiusOfTowel*5, -0.15, -Math.PI*0.1), 2.5);
            cleaningPath.addCleaningPose(new SolarPanelCleaningPose(solarPanel, radiusOfTowel, radiusOfTowel*5, -0.15, -Math.PI*0.1), 5.0);
            
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
