package us.ihmc.robotics.dataStructures;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;

public class FrameVertex2DSupplierList<T extends FramePoint2DReadOnly> extends Vertex2DSupplierList<T> implements FrameVertex2DSupplier
{
   @Override
   public T getVertex(int index)
   {
      return super.getVertex(index);
   }
}
