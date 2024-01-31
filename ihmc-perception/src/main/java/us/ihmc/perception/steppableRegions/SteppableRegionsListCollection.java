package us.ihmc.perception.steppableRegions;

public class SteppableRegionsListCollection
{
   private int discretizations;
   private SteppableRegionsList[] steppableRegions;

   public SteppableRegionsListCollection(int discretizations)
   {
      this.discretizations = discretizations;
      steppableRegions = new SteppableRegionsList[discretizations];
   }


   public SteppableRegionsListCollection(SteppableRegionsList[] steppableRegions)
   {
      this.discretizations = steppableRegions.length;
      this.steppableRegions = steppableRegions;
   }

   public void resizeCollection(int discretizations)
   {
      this.discretizations = discretizations;
      steppableRegions = new SteppableRegionsList[discretizations];
   }

   public void clear()
   {
      for (int i = 0; i < discretizations; i++)
         steppableRegions[i] = null;
   }

   public int getDiscretizations()
   {
      return discretizations;
   }

   public SteppableRegionsList[] getSteppableRegions()
   {
      return steppableRegions;
   }

   public void setSteppableRegions(int yawIndex, SteppableRegionsList steppableRegionsList)
   {
      this.steppableRegions[yawIndex] = steppableRegionsList;
   }

   public SteppableRegionsList getSteppableRegions(int yawIndex)
   {
      return steppableRegions[yawIndex];
   }
}
