package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the generic type class for specifying the continuity of some kind of value between two segments (outlined in {@link MPCValueType) to the MPC core.
 * Can be used to say things like "I want the CoM trajectories to be continuous". These commands can be specified using either constraints (equality only) or
 * objectives (like a soft-constraint, minimizing cost), as defined by the {@link #constraintType}. The representation of the motion function
 * that this value correponds to is defined by the abstract function {@link #getValueType()}}. The derivative order of the command is defined by
 * {@link #getDerivativeOrder()}.
 *
 * The segments are always defined to be sequential, and centered at 0. That means that segment 1 is constrained to be continuous with segment 2, and that
 * a segment always goes from time [0,T]. This can be formulated as f<sub>i</sub>(T<sub>i</sub>) = f<sub>i+1</sub>(0).
 */
public abstract class MPCContinuityCommand implements MPCCommand<MPCContinuityCommand>
{
   /**
    * Contact planes used to generate the motion in the first segment.
    */
   private final List<ContactPlaneHelper> firstSegmentContactPlaneHelpers = new ArrayList<>();
   /**
    * Contact planes used to generate the motion in the first segment.
    */
   private final List<ContactPlaneHelper> secondSegmentContactPlaneHelpers = new ArrayList<>();

   /**
    * Segment number of the first segment. The segment number of the second will always be {@link #firstSegmentNumber} + 1
    */
   private int firstSegmentNumber;
   /**
    * Duration of the first segment. This is where the constraint will be placed for the first segment.
    */
   private double firstSegmentDuration;
   /**
    * Time constant used in the motion functions for the constraint.
    */
   private double omega;
   /**
    * If specifying the constraint as an objective, this is the weight that is placed on satisfying this constraint in the optimization.
    */
   private double weight;
   /**
    * Constraint type for being solved by the MPC.
    */
   private ConstraintType constraintType = ConstraintType.EQUALITY;

   /**
    * @return command type for MPC core.
    */
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.CONTINUITY;
   }

   /**
    * Sets the constraint type to be used when achieving this command (equality constraint, objective, etc.)
    */
   public void setConstraintType(ConstraintType constraintType)
   {
      if (constraintType == ConstraintType.GEQ_INEQUALITY || constraintType == ConstraintType.LEQ_INEQUALITY)
         throw new IllegalArgumentException("Continuity constraints cannot be inequalities.");
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
    * Resets this command
    */
   public void clear()
   {
      firstSegmentContactPlaneHelpers.clear();
      secondSegmentContactPlaneHelpers.clear();
      firstSegmentDuration = Double.NaN;
      firstSegmentNumber = -1;
   }

   /**
    * Adds a contact that the MPC is to use for the first motion segment
    */
   public void addFirstSegmentContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      firstSegmentContactPlaneHelpers.add(contactPlaneHelper);
   }

   /**
    * Adds a contact that the MPC is to use for the second motion segment
    */
   public void addSecondSegmentContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      secondSegmentContactPlaneHelpers.add(contactPlaneHelper);
   }

   /**
    * Sets the segment number that the first segment corresponds to, with the second being {@link #firstSegmentNumber} + 1
    */
   public void setFirstSegmentNumber(int firstSegmentNumber)
   {
      this.firstSegmentNumber = firstSegmentNumber;
   }

   /**
    * Sets the total duration of the first segment, which is where it will be constrained to be equal to the second one.
    */
   public void setFirstSegmentDuration(double firstSegmentDuration)
   {
      this.firstSegmentDuration = firstSegmentDuration;
   }

   /**
    * Sets the time constant for the motion function.
    */
   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   /**
    * Sets the weight for the value objective, when {@link #getConstraintType()}  returns {@link ConstraintType#OBJECTIVE}
    */
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
      return firstSegmentContactPlaneHelpers.size();
   }

   public ContactPlaneHelper getFirstSegmentContactPlaneHelper(int i)
   {
      return firstSegmentContactPlaneHelpers.get(i);
   }

   public int getSecondSegmentNumberOfContacts()
   {
      return secondSegmentContactPlaneHelpers.size();
   }

   public ContactPlaneHelper getSecondSegmentContactPlaneHelper(int i )
   {
      return secondSegmentContactPlaneHelpers.get(i);
   }
}
