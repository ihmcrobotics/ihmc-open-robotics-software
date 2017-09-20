package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

public class GroundAirDescriptor
{
   private Float height;
   private Float minClearHeight;
   public GroundAirDescriptor(Float height, Float minClearHeight)
   {
      this.height=height;
      this.minClearHeight=height;
   }
   public Float getHeight()
   {
      return height;
   }
   public void setHeight(Float height)
   {
      this.height = height;
   }
   public Float getMinClearHeight()
   {
      return minClearHeight;
   }
   public void setMinClearHeight(Float minClearHeight)
   {
      this.minClearHeight = minClearHeight;
   }
}
