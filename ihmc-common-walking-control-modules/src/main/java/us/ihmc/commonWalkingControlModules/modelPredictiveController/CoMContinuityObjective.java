package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class CoMContinuityObjective
{
   private final FramePoint3D objective = new FramePoint3D();
   private CoefficientJacobianMatrixHelper firstSegmentJacobianMatrixHelper;
   private CoefficientJacobianMatrixHelper secondSegmentJacobianMatrixHelper;

   private int derivativeOrder;
   private int firstSegmentNumber;
   private double firstSegmentDuration;
   private double omega;

   public void setFirstSegmentJacobianMatrixHelper(CoefficientJacobianMatrixHelper firstSegmentJacobianMatrixHelper)
   {
      this.firstSegmentJacobianMatrixHelper = firstSegmentJacobianMatrixHelper;
   }

   public void setSecondSegmentJacobianMatrixHelper(CoefficientJacobianMatrixHelper secondSegmentJacobianMatrixHelper)
   {
      this.secondSegmentJacobianMatrixHelper = secondSegmentJacobianMatrixHelper;
   }
   
   public void setDerivativeOrder(int derivativeOrder)
   {
      this.derivativeOrder = derivativeOrder;
   }

   public void setFirstSegmentNumber(int firstSegmentNumber)
   {
      this.firstSegmentNumber = firstSegmentNumber;
   }

   public void setFirstSegmentDuration(double firstSegmentDuration)
   {
      this.firstSegmentDuration = firstSegmentDuration;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public int getFirstSegmentNumber()
   {
      return firstSegmentNumber;
   }

   public int getDerivativeOrder()
   {
      return derivativeOrder;
   }

   public double getFirstSegmentDuration()
   {
      return firstSegmentDuration;
   }

   public double getOmega()
   {
      return omega;
   }

   public FramePoint3DReadOnly getObjective()
   {
      return objective;
   }

   public CoefficientJacobianMatrixHelper getFirstSegmentJacobianMatrixHelper()
   {
      return firstSegmentJacobianMatrixHelper;
   }

   public CoefficientJacobianMatrixHelper getSecondSegmentJacobianMatrixHelper()
   {
      return secondSegmentJacobianMatrixHelper;
   }
}
