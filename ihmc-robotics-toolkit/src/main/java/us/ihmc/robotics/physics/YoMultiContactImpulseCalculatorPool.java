package us.ihmc.robotics.physics;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.lists.SupplierBuilder;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoMultiContactImpulseCalculatorPool
{
   private final RecyclingArrayList<YoMultiContactImpulseCalculator> pool;

   public YoMultiContactImpulseCalculatorPool(int initialCapacity, ReferenceFrame rootFrame, YoRegistry registry)
   {
      pool = new RecyclingArrayList<>(initialCapacity,
                                      SupplierBuilder.indexedSupplier(identifier -> new YoMultiContactImpulseCalculator(identifier, rootFrame, registry)));
      pool.clear();
   }

   public YoMultiContactImpulseCalculator nextAvailable()
   {
      return pool.add();
   }

   public void clear()
   {
      for (int i = 0; i < pool.size(); i++)
      {
         pool.get(i).clear();
      }
      pool.clear();
   }
}
