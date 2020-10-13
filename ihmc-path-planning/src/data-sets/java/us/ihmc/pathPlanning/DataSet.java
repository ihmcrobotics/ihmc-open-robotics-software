package us.ihmc.pathPlanning;

import us.ihmc.robotics.geometry.PlanarRegionsList;

public class DataSet
{
   private final String name;
   private final PlanarRegionsList planarRegionsList;
   private PlannerInput plannerInput = null;

   public DataSet(String name, PlanarRegionsList planarRegionsList)
   {
      this.name = name;
      this.planarRegionsList = planarRegionsList;
   }

   public String getName()
   {
      return name;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public boolean hasPlannerInput()
   {
      return plannerInput != null;
   }

   public PlannerInput getPlannerInput()
   {
      return plannerInput;
   }

   public void setPlannerInput(PlannerInput plannerInput)
   {
      this.plannerInput = plannerInput;
   }

   @Override
   public String toString()
   {
      return name;
   }
}
