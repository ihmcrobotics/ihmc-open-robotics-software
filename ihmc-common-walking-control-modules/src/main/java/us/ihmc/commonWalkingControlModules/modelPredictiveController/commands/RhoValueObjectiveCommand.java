package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoefficientJacobianMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactStateMagnitudeToForceMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

import java.util.ArrayList;
import java.util.List;

public class RhoValueObjectiveCommand implements MPCCommand<RhoValueObjectiveCommand>
{
   private final List<CoefficientJacobianMatrixHelper> jacobianMatrixHelpers = new ArrayList<>();

   private int segmentNumber;
   private double timeOfObjective;
   private double omega;

   private ConstraintType constraintType;

   private double objective = 0.0;

   public void clear()
   {
      jacobianMatrixHelpers.clear();
   }

   public void addCoefficientJacobianMatrixHelper(CoefficientJacobianMatrixHelper jacobianMatrixHelper)
   {
      this.jacobianMatrixHelpers.add(jacobianMatrixHelper);
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
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

   public void setObjective(double objective)
   {
      this.objective = objective;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
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

   public double getObjective()
   {
      return objective;
   }

   public CoefficientJacobianMatrixHelper getCoefficientJacobianMatrixHelper(int i)
   {
      return jacobianMatrixHelpers.get(i);
   }

   public int getNumberOfContacts()
   {
      return jacobianMatrixHelpers.size();
   }

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.RHO_VALUE;
   }
}
