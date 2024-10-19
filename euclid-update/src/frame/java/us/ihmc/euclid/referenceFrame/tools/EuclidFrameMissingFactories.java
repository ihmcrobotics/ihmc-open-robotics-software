package us.ihmc.euclid.referenceFrame.tools;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tuple3D.interfaces.*;

import static us.ihmc.euclid.tools.EuclidHashCodeTools.toIntHashCode;

public class EuclidFrameMissingFactories
{
   public static FramePoint3DReadOnly newLinkedFramePoint3DReadOnly(ReferenceFrameHolder referenceFrameHolder, Tuple3DReadOnly originalPoint)
   {
      return new FramePoint3DReadOnly()
      {
         @Override
         public double getX()
         {
            return originalPoint.getX();
         }

         @Override
         public double getY()
         {
            return originalPoint.getY();
         }

         @Override
         public double getZ()
         {
            return originalPoint.getZ();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FramePoint3DReadOnly)
               return equals((FramePoint3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }


   public static FrameVector3DReadOnly newLinkedFrameVector3DReadOnly(ReferenceFrameHolder referenceFrameHolder, Vector3DReadOnly originalVector)
   {
      return new FrameVector3DReadOnly()
      {
         @Override
         public double getX()
         {
            return originalVector.getX();
         }

         @Override
         public double getY()
         {
            return originalVector.getY();
         }

         @Override
         public double getZ()
         {
            return originalVector.getZ();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return referenceFrameHolder.getReferenceFrame();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof FrameVector3DReadOnly)
               return equals((FrameVector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()), getReferenceFrame());
         }

         @Override
         public String toString()
         {
            return EuclidFrameIOTools.getFrameTuple3DString(this);
         }
      };
   }

}
