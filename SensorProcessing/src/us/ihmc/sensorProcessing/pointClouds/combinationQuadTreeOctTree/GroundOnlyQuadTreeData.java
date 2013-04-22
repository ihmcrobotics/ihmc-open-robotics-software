package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

public class GroundOnlyQuadTreeData
{
   private final boolean isStuffAboveMe;
   public GroundOnlyQuadTreeData(boolean stuffAbove)
   {
      this.isStuffAboveMe=stuffAbove;
   }
   public boolean getIsStuffAboveMe()
   {
      return isStuffAboveMe;
   }
}