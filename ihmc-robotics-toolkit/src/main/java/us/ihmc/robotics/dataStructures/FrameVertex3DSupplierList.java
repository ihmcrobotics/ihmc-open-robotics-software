package us.ihmc.robotics.dataStructures;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;

public class FrameVertex3DSupplierList<T extends FramePoint3DReadOnly> extends Vertex3DSupplierList<T> implements FrameVertex3DSupplier
{
   @Override
   public T getVertex(int index)
   {
      return super.getVertex(index);
   }
}
