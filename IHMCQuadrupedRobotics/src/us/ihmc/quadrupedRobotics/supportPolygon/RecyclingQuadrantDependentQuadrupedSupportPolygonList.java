package us.ihmc.quadrupedRobotics.supportPolygon;

import us.ihmc.robotics.robotSide.RecyclingQuadrantDependentList;

public class RecyclingQuadrantDependentQuadrupedSupportPolygonList extends RecyclingQuadrantDependentList<QuadrupedSupportPolygon>
{

   public RecyclingQuadrantDependentQuadrupedSupportPolygonList()
   {
      super(new GenericTypeAdapter<QuadrupedSupportPolygon>()
      {
         @Override
         public QuadrupedSupportPolygon makeANewV()
         {
            return new QuadrupedSupportPolygon();
         }

         @Override
         public void setAV(QuadrupedSupportPolygon newV, QuadrupedSupportPolygon setThisV)
         {
            setThisV.set(newV);
         }
      });
   }

}
