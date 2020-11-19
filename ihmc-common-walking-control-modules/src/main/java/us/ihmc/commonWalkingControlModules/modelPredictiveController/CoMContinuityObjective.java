package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class CoMContinuityObjective implements MPCCommand<CoMContinuityObjective>
{
   private final List<CoefficientJacobianMatrixHelper> firstSegmentJacobianMatrixHelpers = new ArrayList<>();
   private final List<CoefficientJacobianMatrixHelper> secondSegmentJacobianMatrixHelpers = new ArrayList<>();

   private final List<ContactStateMagnitudeToForceMatrixHelper> firstRhoToForceMatrixHelpers = new ArrayList<>();
   private final List<ContactStateMagnitudeToForceMatrixHelper> secondRhoToForceMatrixHelpers = new ArrayList<>();

   private int derivativeOrder;
   private int firstSegmentNumber;
   private double firstSegmentDuration;
   private double omega;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.CONTINUITY;
   }

   public void clear()
   {
      firstSegmentJacobianMatrixHelpers.clear();
      secondSegmentJacobianMatrixHelpers.clear();

      firstRhoToForceMatrixHelpers.clear();
      secondRhoToForceMatrixHelpers.clear();
   }

   public void addFirstSegmentRhoToForceMatrixHelper(ContactStateMagnitudeToForceMatrixHelper firstRhoToForceMatrixHelper)
   {
      firstRhoToForceMatrixHelpers.add(firstRhoToForceMatrixHelper);
   }

   public void addSecondSegmentRhoToForceMatrixHelper(ContactStateMagnitudeToForceMatrixHelper secondRhoToForceMatrixHelper)
   {
      secondRhoToForceMatrixHelpers.add(secondRhoToForceMatrixHelper);
   }

   public void addFirstSegmentJacobianMatrixHelper(CoefficientJacobianMatrixHelper firstSegmentJacobianMatrixHelper)
   {
      firstSegmentJacobianMatrixHelpers.add(firstSegmentJacobianMatrixHelper);
   }

   public void addSecondSegmentJacobianMatrixHelper(CoefficientJacobianMatrixHelper secondSegmentJacobianMatrixHelper)
   {
      secondSegmentJacobianMatrixHelpers.add(secondSegmentJacobianMatrixHelper);
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

   public int getFirstSegmentNumberOfContacts()
   {
      return firstRhoToForceMatrixHelpers.size();
   }

   public CoefficientJacobianMatrixHelper getFirstSegmentCoefficientJacobianMatrixHelper(int i)
   {
      return firstSegmentJacobianMatrixHelpers.get(i);
   }

   public ContactStateMagnitudeToForceMatrixHelper getFirstSegmentRhoToForceMatrixHelper(int i)
   {
      return firstRhoToForceMatrixHelpers.get(i);
   }

   public int getSecondSegmentNumberOfContacts()
   {
      return secondRhoToForceMatrixHelpers.size();
   }

   public CoefficientJacobianMatrixHelper getSecondSegmentCoefficientJacobianMatrixHelper(int i)
   {
      return secondSegmentJacobianMatrixHelpers.get(i);
   }

   public ContactStateMagnitudeToForceMatrixHelper getSecondSegmentRhoToForceMatrixHelper(int i)
   {
      return secondRhoToForceMatrixHelpers.get(i);
   }
}
