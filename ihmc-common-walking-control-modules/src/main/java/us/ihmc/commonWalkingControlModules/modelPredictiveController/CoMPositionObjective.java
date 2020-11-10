package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class CoMPositionObjective
{
   private final FramePoint3D positionObjective = new FramePoint3D();
   private CoefficientJacobianMatrixHelper jacobianMatrixHelper;

   private int segmentNumber;
   private double timeOfObjective;
   private double omega;

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public double getTimeOfObjective()
   {
      return timeOfObjective;
   }

   public double getOmega()
   {
      return omega;
   }

   public FramePoint3DReadOnly getPositionObjective()
   {
      return positionObjective;
   }

   public CoefficientJacobianMatrixHelper getJacobianMatrixHelper()
   {
      return jacobianMatrixHelper;
   }
}
