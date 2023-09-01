package us.ihmc.robotics;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MecanoIOTools;

public class MecanoMissingTools
{
   public static WrenchReadOnly newLinkedWrenchReadOnly(ReferenceFrameHolder bodyFrame, ReferenceFrameHolder expressedInFrame, DMatrixRMaj wrenchMatrixSource)
   {
      FrameVector3DReadOnly angularPart = EuclidCoreMissingTools.newLinkedFrameVector3DReadOnly(expressedInFrame, 0, wrenchMatrixSource);
      FrameVector3DReadOnly linearPart = EuclidCoreMissingTools.newLinkedFrameVector3DReadOnly(expressedInFrame, 3, wrenchMatrixSource);

      return new WrenchReadOnly()
      {
         @Override
         public ReferenceFrame getBodyFrame()
         {
            return bodyFrame.getReferenceFrame();
         }

         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return angularPart.getReferenceFrame();
         }

         @Override
         public FrameVector3DReadOnly getAngularPart()
         {
            return angularPart;
         }

         @Override
         public FrameVector3DReadOnly getLinearPart()
         {
            return linearPart;
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof WrenchReadOnly)
               return equals((WrenchReadOnly) object);
            else
               return false;
         }

         @Override
         public void get(DMatrix matrixToPack)
         {
            matrixToPack.set(wrenchMatrixSource);
         }

         @Override
         public String toString()
         {
            return MecanoIOTools.getWrenchString(this);
         }
      };
   }
}
