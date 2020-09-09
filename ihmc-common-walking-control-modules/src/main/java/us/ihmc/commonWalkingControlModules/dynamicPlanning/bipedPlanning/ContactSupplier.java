package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import gnu.trove.list.TDoubleList;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;

import java.util.List;

public interface ContactSupplier
{
   List<? extends ConvexPolygon2DReadOnly> getSupportPolygons();

   TDoubleList getSupportTimes();
}
