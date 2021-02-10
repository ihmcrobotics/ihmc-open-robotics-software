package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

/**
 * This is the generic type class for specifying some kind of value (outlined in {@link MPCValueType) to the MPC core.
 * Can be used to say things like "I want the DCM to be at this position at this time". These commands can be specified using either constraints (equality or
 * inequality) or objectives (like a soft-constraint, minimizing cost), as defined by the {@link #constraintType}. The representation of the motion function
 * that this value correponds to is defined by the abstract function {@link #getValueType()}}. The derivative order of the command is defined by {@link #getDerivativeOrder()}.
 */
public abstract class MPCValueCommand implements MPCCommand<MPCValueCommand>
{
   /**
    * Holder for the objective value for this command.
    */
   private final FramePoint3D objective = new FramePoint3D();
   /**
    * Defines the contact planes to be used to achieve the objective.
    */
   private final List<ContactPlaneHelper> contactPlaneHelpers = new ArrayList<>();

   /**
    * What segment number this objective corresponds to.
    */
   private int segmentNumber;
   /**
    * Time in the segment that this objective corresponds to.
    */
   private double timeOfObjective;
   /**
    * Time constant used in the CoM function for this command.
    */
   private double omega;
   /**
    * If specifying the value as an objective, this is the weight that is placed on satisfying this objective in the optimization.
    */
   private double weight;
   /**
    * Constraint type for being solved by the MPC.
    */
   private ConstraintType constraintType = ConstraintType.OBJECTIVE;

   /**
    * Consumer for the computed cost to go on the output of the MPC function.
    */
   private DoubleConsumer costToGoConsumer;

   /**
    * @return command type for MPC core.
    */
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.VALUE;
   }

   /**
    * Sets the constraint type to be used when achieving this command (equality constraint, objective, etc.)
    */
   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   /**
    * Gets the constraint type to be used when achieving this command (equality constraint, objective, etc.)
    */
   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   /**
    * Gets the derivative order of this command, where 0=position, 1=velocity, etc.
    */
   public abstract int getDerivativeOrder();

   /**
    * Gets the value representation of this command, i.e. whether it's CoM, DCM, VRP
    */
   public abstract MPCValueType getValueType();

   /**
    * Removes the consumers and clears the contact planes, resetting the command.
    */
   public void clear()
   {
      costToGoConsumer = null;
      segmentNumber = -1;
      timeOfObjective = Double.NaN;
      objective.setToNaN();
      contactPlaneHelpers.clear();
   }

   /**
    * Sets the weight for the value objective, when {@link #getConstraintType()}  returns {@link ConstraintType#OBJECTIVE}
    */
   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   /**
    * Adds a contact that the MPC can use to try and achieve the desired objective
    */
   public void addContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
   }

   /**
    * Sets the actual value that is trying to be achieved by the MPC at the specified time and segment.
    */
   public void setObjective(FrameTuple3DReadOnly objective)
   {
      this.objective.set(objective);
   }

   /**
    * Sets the segment number that this command corresponds to.
    */
   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   /**
    * Sets the time at which the MPC is to try and achieve this command.
    */
   public void setTimeOfObjective(double timeOfObjective)
   {
      this.timeOfObjective = timeOfObjective;
   }

   /**
    * Sets the time constant for the motion function.
    */
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

   public void setCostToGoConsumer(DoubleConsumer costToGoConsumer)
   {
      this.costToGoConsumer = costToGoConsumer;
   }

   public void setCostToGo(double costToGo)
   {
      if (costToGoConsumer != null)
         costToGoConsumer.accept(costToGo);
   }
}
