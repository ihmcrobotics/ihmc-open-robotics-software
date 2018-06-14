package us.ihmc.robotics.dataStructures;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;

public class FrameVertex3DSupplierList<T extends FramePoint3DReadOnly> extends Vertex3DSupplierList<T> implements FrameVertex3DSupplier
{
   @Override
   public FramePoint3DReadOnly getVertex(int index)
   {
      return (FramePoint3DReadOnly) super.getVertex(index);
   }
}
