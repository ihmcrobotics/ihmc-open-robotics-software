package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoMTrajectoryModelPredictiveController;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoefficientJacobianMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactStateMagnitudeToForceMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;

import java.util.ArrayList;
import java.util.List;

public abstract class CoMContinuityCommand implements MPCCommand<CoMContinuityCommand>
{
   private final List<CoefficientJacobianMatrixHelper> firstSegmentJacobianMatrixHelpers = new ArrayList<>();
   private final List<CoefficientJacobianMatrixHelper> secondSegmentJacobianMatrixHelpers = new ArrayList<>();

   private final List<ContactStateMagnitudeToForceMatrixHelper> firstRhoToForceMatrixHelpers = new ArrayList<>();
   private final List<ContactStateMagnitudeToForceMatrixHelper> secondRhoToForceMatrixHelpers = new ArrayList<>();

   private int firstSegmentNumber;
   private double firstSegmentDuration;
   private double omega;
   private double weight = CoMTrajectoryModelPredictiveController.MEDIUM_WEIGHT;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.CONTINUITY;
   }

   public abstract int getDerivativeOrder();

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

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public int getFirstSegmentNumber()
   {
      return firstSegmentNumber;
   }

   public double getFirstSegmentDuration()
   {
      return firstSegmentDuration;
   }

   public double getOmega()
   {
      return omega;
   }

   public double getWeight()
   {
      return weight;
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
