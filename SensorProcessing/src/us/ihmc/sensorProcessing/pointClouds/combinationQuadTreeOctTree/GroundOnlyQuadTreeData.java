package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

public class GroundOnlyQuadTreeData
{
   private boolean isStuffAboveMe;
   private double height;
   public GroundOnlyQuadTreeData()
   {
      this.isStuffAboveMe=false;
      this.height=Double.NaN;
   }
   public boolean getIsStuffAboveMe()
   {
      return isStuffAboveMe;
   }
   public double getHeight()
   {
      return height;
   }
   public boolean isHeightValid()
   {
      return !Double.isNaN(height);
   }
   public void setIsStuffAboveMe(boolean isStuffAbove)
   {
      this.isStuffAboveMe=isStuffAbove;
   }
   public void setHeight(double height)
   {
      this.height=height;
   }
}