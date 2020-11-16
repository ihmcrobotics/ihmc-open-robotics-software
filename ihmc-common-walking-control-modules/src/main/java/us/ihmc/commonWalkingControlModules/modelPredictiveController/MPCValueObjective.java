package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class MPCValueObjective
{
   private final FramePoint3D objective = new FramePoint3D();
   private final List<CoefficientJacobianMatrixHelper> jacobianMatrixHelpers = new ArrayList<>();
   private final List<ContactStateMagnitudeToForceMatrixHelper> rhoToForceMatrixHelpers = new ArrayList<>();

   private int derivativeOrder;
   private int segmentNumber;
   private double timeOfObjective;
   private double omega;

   private MPCValueType valueType = MPCValueType.COM;

   public void clear()
   {
      rhoToForceMatrixHelpers.clear();
      jacobianMatrixHelpers.clear();
   }

   public void addRhoToForceMatrixHelper(ContactStateMagnitudeToForceMatrixHelper helper)
   {
      rhoToForceMatrixHelpers.add(helper);
   }

   public void addJacobianMatrixHelper(CoefficientJacobianMatrixHelper jacobianMatrixHelper)
   {
      jacobianMatrixHelpers.add(jacobianMatrixHelper);
   }

   public void setValueType(MPCValueType valueType)
   {
      this.valueType = valueType;
   }

   public void setObjective(FrameTuple3DReadOnly objective)
   {
      this.objective.set(objective);
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

   public MPCValueType getValueType()
   {
      return valueType;
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

   public FrameTuple3DReadOnly getObjective()
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
