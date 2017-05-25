package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import us.ihmc.commons.PrintTools;
import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.solarpanelmotion.SolarPanelPath;

public class SolarPanelCleaningInfo
{
   private static SolarPanelPath cleaningPath;
   private static DegreesOfRedundancy cleaningPoseDegrees;
   private static CleaningPathType cleaningType;
   
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
   
   public static void setCleaningPath(SolarPanelPath path)
   {
      for(int i=0;i<path.getNumerOfLinearPath();i++)
      {
         PrintTools.info(""+i+" "+path.getLinearPath().get(i).getMotionStartTime() +" "+path.getLinearPath().get(i).getMotionEndTime());
      }
      cleaningPath = path;
   }
   
   public static SolarPanelPath getCleaningPath()
   {
      return cleaningPath;
   }
   
   public static void setDegreesOfRedundancy(DegreesOfRedundancy degrees)
   {
      cleaningPoseDegrees = degrees;
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
   
   public static void setCleaningPathType(CleaningPathType type)
   {
      cleaningType = type;
   }
   
   public static CleaningPathType getCleaningType()
   {
      return cleaningType;
   }
}
