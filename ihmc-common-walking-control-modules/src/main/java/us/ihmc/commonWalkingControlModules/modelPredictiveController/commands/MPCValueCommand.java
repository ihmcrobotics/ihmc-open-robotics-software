package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoefficientJacobianMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactStateMagnitudeToForceMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public abstract class MPCValueCommand implements MPCCommand<MPCValueCommand>
{
   private final FramePoint3D objective = new FramePoint3D();
   private final List<CoefficientJacobianMatrixHelper> jacobianMatrixHelpers = new ArrayList<>();
   private final List<ContactStateMagnitudeToForceMatrixHelper> rhoToForceMatrixHelpers = new ArrayList<>();

   private int segmentNumber;
   private double timeOfObjective;
   private double omega;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.VALUE;
   }

   public abstract int getDerivativeOrder();

   public abstract MPCValueType getValueType();

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

   public void setObjective(FrameTuple3DReadOnly objective)
   {
      this.objective.set(objective);
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
