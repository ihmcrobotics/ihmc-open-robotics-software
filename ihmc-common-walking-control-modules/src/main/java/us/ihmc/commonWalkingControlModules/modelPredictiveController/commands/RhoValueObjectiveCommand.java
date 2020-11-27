package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoefficientJacobianMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactStateMagnitudeToForceMatrixHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;

import java.util.ArrayList;
import java.util.List;

public class RhoValueObjectiveCommand implements MPCCommand<RhoValueObjectiveCommand>
{
   private final List<ContactPlaneHelper> contactPlaneHelpers = new ArrayList<>();

   private int segmentNumber;
   private double timeOfObjective;
   private double omega;

   private ConstraintType constraintType;

   private double objective = 0.0;
   private final DMatrixRMaj objectiveVector = new DMatrixRMaj(0, 0);
   private boolean useScalarObjective = true;

   public void clear()
   {
      contactPlaneHelpers.clear();
   }

   public void addContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
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

   public void setScalarObjective(double objective)
   {
      this.objective = objective;
   }

   public void setUseScalarObjective(boolean useScalarObjective)
   {
      this.useScalarObjective = useScalarObjective;
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

   public boolean getUseScalarObjective()
   {
      return useScalarObjective;
   }

   public double getScalarObjective()
   {
      return objective;
   }

   public DMatrixRMaj getObjectiveVector()
   {
      return objectiveVector;
   }

   public ContactPlaneHelper getContactPlaneHelper(int i)
   {
      return contactPlaneHelpers.get(i);
   }

   public int getNumberOfContacts()
   {
      return contactPlaneHelpers.size();
   }

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.RHO_VALUE;
   }
}
