package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface CoMHeightPartialDerivativesDataBasics extends CoMHeightPartialDerivativesDataReadOnly
{
   default void set(CoMHeightPartialDerivativesDataReadOnly centerOfMassHeightPartialDerivativesData)
   {
      setCoMHeight(centerOfMassHeightPartialDerivativesData.getFrameOfCoMHeight(), centerOfMassHeightPartialDerivativesData.getComHeight());
      setPartialDzDx(centerOfMassHeightPartialDerivativesData.getPartialDzDx());
      setPartialDzDy(centerOfMassHeightPartialDerivativesData.getPartialDzDy());
      setPartialD2zDx2(centerOfMassHeightPartialDerivativesData.getPartialD2zDx2());
      setPartialD2zDy2(centerOfMassHeightPartialDerivativesData.getPartialD2zDy2());
      setPartialD2zDxDy(centerOfMassHeightPartialDerivativesData.getPartialD2zDxDy());
      setPartialD3zDx3(centerOfMassHeightPartialDerivativesData.getPartialD3zDx3());
      setPartialD3zDy3(centerOfMassHeightPartialDerivativesData.getPartialD3zDy3());
      setPartialD3zDx2Dy(centerOfMassHeightPartialDerivativesData.getPartialD3zDx2Dy());
      setPartialD3zDxDy2(centerOfMassHeightPartialDerivativesData.getPartialD3zDxDy2());
   }

   void setCoMHeight(ReferenceFrame referenceFrame, double comHeight);

   void setPartialDzDx(double partialDzDx);

   void setPartialDzDy(double partialDzDy);

   void setPartialD2zDx2(double partialD2zDx2);

   void setPartialD2zDy2(double partialD2zDy2);

   void setPartialD2zDxDy(double partialD2zDxDy);

   void setPartialD3zDx3(double partialD3zDx3);

   void setPartialD3zDy3(double partialD3zDy3);

   void setPartialD3zDx2Dy(double partialD3zDx2Dy);

   void setPartialD3zDxDy2(double partialD3zDxDy2);
}
