package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class CoMValueObjective
{
   private final FramePoint3D objective = new FramePoint3D();
   private CoefficientJacobianMatrixHelper jacobianMatrixHelper;

   private int derivativeOrder;
   private int segmentNumber;
   private double timeOfObjective;
   private double omega;

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public int getDerivativeOrder()
   {
      return derivativeOrder;
   }

   public double getTimeOfObjective()
   {
      return timeOfObjective;
   }

   public double getOmega()
   {
      return omega;
   }

   public FramePoint3DReadOnly getObjective()
   {
      return objective;
   }

   public CoefficientJacobianMatrixHelper getJacobianMatrixHelper()
   {
      return jacobianMatrixHelper;
   }
}
