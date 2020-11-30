package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.JumpingFootControlModule;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public abstract class MPCValueCommand implements MPCCommand<MPCValueCommand>
{
   private final FramePoint3D objective = new FramePoint3D();
   private final List<ContactPlaneHelper> contactPlaneHelpers = new ArrayList<>();

   private int segmentNumber;
   private double timeOfObjective;
   private double omega;
   private double weight = CoMTrajectoryModelPredictiveController.MEDIUM_WEIGHT;
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.VALUE;
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   public abstract int getDerivativeOrder();

   public abstract MPCValueType getValueType();

   public void clear()
   {
      contactPlaneHelpers.clear();
   }

   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void addContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
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

   public double getWeight()
   {
      return weight;
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

   public int getNumberOfContacts()
   {
      return contactPlaneHelpers.size();
   }

   public ContactPlaneHelper getContactPlaneHelper(int i)
   {
      return contactPlaneHelpers.get(i);
   }
}
