package us.ihmc.commonWalkingControlModules.heightPlanning;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface CoMHeightPartialDerivativesDataReadOnly
{
   ReferenceFrame getFrameOfCoMHeight();

   double getComHeight();

   double getPartialDzDx();

   double getPartialDzDy();

   double getPartialD2zDx2();

   double getPartialD2zDy2();

   double getPartialD2zDxDy();

   double getPartialD3zDx3();

   double getPartialD3zDy3();

   double getPartialD3zDx2Dy();

   double getPartialD3zDxDy2();
}
