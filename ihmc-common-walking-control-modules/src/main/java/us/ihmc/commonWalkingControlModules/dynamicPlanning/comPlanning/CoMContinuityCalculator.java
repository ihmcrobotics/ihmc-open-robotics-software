package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.ejml.data.DMatrix;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.List;

public interface CoMContinuityCalculator
{
   void setInitialCoMPosition(FramePoint3DReadOnly initialCoMPosition);

   void setInitialCoMVelocity(FrameVector3DReadOnly initialCoMVelocity);

   void setFinalICPToAchieve(FramePoint3DReadOnly finalICPToAchieve);

   boolean solve(List<? extends ContactStateProvider> contactSequence);

   int getDepthForCalculation();

   void getXCoefficientOverrides(DMatrix xCoefficientVectorToPack);

   void getYCoefficientOverrides(DMatrix yCoefficientVectorToPack);

   void getZCoefficientOverrides(DMatrix zCoefficientVectorToPack);
}
