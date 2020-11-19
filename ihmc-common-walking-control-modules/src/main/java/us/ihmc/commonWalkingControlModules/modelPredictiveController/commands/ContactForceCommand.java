package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoefficientJacobianMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactStateMagnitudeToForceMatrixHelper;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

public class ContactForceCommand
{
   private CoefficientJacobianMatrixHelper jacobianMatrixHelper;
   private ContactStateMagnitudeToForceMatrixHelper rhoToForceMatrixHelper;

   private int segmentNumber;
   private double timeOfObjective;
   private double omega;

   private ConstraintType constraintType;

   private final FrameVector3D objective = new FrameVector3D();
   private final SelectionMatrix3D selectionMatrix = new SelectionMatrix3D();

   public void setCoefficientJacobianMatrixHelper(CoefficientJacobianMatrixHelper jacobianMatrixHelper)
   {
      this.jacobianMatrixHelper = jacobianMatrixHelper;
   }

   public void setRhoToForceMatrixHelper(ContactStateMagnitudeToForceMatrixHelper rhoToForceMatrixHelper)
   {
      this.rhoToForceMatrixHelper = rhoToForceMatrixHelper;
   }

   public FrameVector3DBasics getObjective()
   {
      return objective;
   }

   public SelectionMatrix3D getSelectionMatrix()
   {
      return selectionMatrix;
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
}
