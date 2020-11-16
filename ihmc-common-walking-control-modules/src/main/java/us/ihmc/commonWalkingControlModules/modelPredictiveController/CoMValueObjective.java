package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class CoMValueObjective
{
   private final FramePoint3D objective = new FramePoint3D();
   private final List<CoefficientJacobianMatrixHelper> jacobianMatrixHelpers = new ArrayList<>();
   private final List<ContactStateMagnitudeToForceMatrixHelper> rhoToForceMatrixHelpers = new ArrayList<>();

   private int derivativeOrder;
   private int segmentNumber;
   private double timeOfObjective;
   private double omega;

   public void addRhoToForceMatrixHelper(ContactStateMagnitudeToForceMatrixHelper helper)
   {
      rhoToForceMatrixHelpers.add(helper);
   }

   public void addJacobianMatrixHelper(CoefficientJacobianMatrixHelper jacobianMatrixHelper)
   {
      jacobianMatrixHelpers.add(jacobianMatrixHelper);
   }

   public void setDerivativeOrder(int derivativeOrder)
   {
      this.derivativeOrder = derivativeOrder;
   }

   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   public void setTimeOfObjective(double timeOfObjective)
   {
      this.timeOfObjective = timeOfObjective;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

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

   public CoefficientJacobianMatrixHelper getCoefficientJacobianMatrixHelper(int i)
   {
      return jacobianMatrixHelpers.get(i);
   }

   public int getNumberOfContacts()
   {
      return rhoToForceMatrixHelpers.size();
   }

   public ContactStateMagnitudeToForceMatrixHelper getRhoToForceMatrixHelper(int i)
   {
      return rhoToForceMatrixHelpers.get(i);
   }
}
