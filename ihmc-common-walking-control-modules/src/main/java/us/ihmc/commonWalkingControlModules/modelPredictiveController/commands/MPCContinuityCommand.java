package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;

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
   private int commandId;
   /**
    * Contact planes used to generate the motion in the first segment.
    */
   private final List<MPCContactPlane> firstSegmentContactPlaneHelpers = new ArrayList<>();
   /**
    * Contact planes used to generate the motion in the first segment.
    */
   private final List<MPCContactPlane> secondSegmentContactPlaneHelpers = new ArrayList<>();

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
   public void addFirstSegmentContactPlaneHelper(MPCContactPlane contactPlaneHelper)
   {
      firstSegmentContactPlaneHelpers.add(contactPlaneHelper);
   }

   /**
    * Adds a contact that the MPC is to use for the second motion segment
    */
   public void addSecondSegmentContactPlaneHelper(MPCContactPlane contactPlaneHelper)
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

   public MPCContactPlane getFirstSegmentContactPlaneHelper(int i)
   {
      return firstSegmentContactPlaneHelpers.get(i);
   }

   public int getSecondSegmentNumberOfContacts()
   {
      return secondSegmentContactPlaneHelpers.size();
   }

   public MPCContactPlane getSecondSegmentContactPlaneHelper(int i )
   {
      return secondSegmentContactPlaneHelpers.get(i);
   }

   @Override
   public void set(MPCContinuityCommand other)
   {
      if (getValueType() != other.getValueType())
         throw new IllegalArgumentException("Cannot set a command of type " + getValueType() + " from a command of type " + other.getValueType());
      if (getDerivativeOrder() != other.getDerivativeOrder())
         throw new IllegalArgumentException("Cannot set a command of derivative order " + getDerivativeOrder() + " from a command of derivative order " + other.getDerivativeOrder());

      clear();
      setCommandId(other.getCommandId());
      setFirstSegmentNumber(other.getFirstSegmentNumber());
      setFirstSegmentDuration(other.getFirstSegmentDuration());
      setOmega(other.getOmega());
      setWeight(other.getWeight());
      setConstraintType(other.getConstraintType());
      for (int i = 0; i < other.getFirstSegmentNumberOfContacts(); i++)
         addFirstSegmentContactPlaneHelper(other.getFirstSegmentContactPlaneHelper(i));
      for (int i = 0; i < other.getSecondSegmentNumberOfContacts(); i++)
         addSecondSegmentContactPlaneHelper(other.getSecondSegmentContactPlaneHelper(i));
   }

   @Override
   public void setCommandId(int id)
   {
      commandId = id;
   }

   @Override
   public int getCommandId()
   {
      return commandId;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MPCContinuityCommand)
      {
         MPCContinuityCommand other = (MPCContinuityCommand) object;
         if (commandId != other.commandId)
            return false;
         if (constraintType != other.constraintType)
            return false;
         if (getValueType() != other.getValueType())
            return false;
         if (getDerivativeOrder() != other.getDerivativeOrder())
            return false;
         if (firstSegmentNumber != other.firstSegmentNumber)
            return false;
         if (firstSegmentDuration != other.firstSegmentDuration)
            return false;
         if (omega != other.omega)
            return false;
         if (weight != other.weight)
            return false;
         if (firstSegmentContactPlaneHelpers.size() != other.firstSegmentContactPlaneHelpers.size())
            return false;
         for (int i = 0; i < firstSegmentContactPlaneHelpers.size(); i++)
         {
            if (!firstSegmentContactPlaneHelpers.get(i).equals(other.firstSegmentContactPlaneHelpers.get(i)))
               return false;
         }
         if (secondSegmentContactPlaneHelpers.size() != other.secondSegmentContactPlaneHelpers.size())
            return false;
         for (int i = 0; i < secondSegmentContactPlaneHelpers.size(); i++)
         {
            if (!secondSegmentContactPlaneHelpers.get(i).equals(other.secondSegmentContactPlaneHelpers.get(i)))
               return false;
         }
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public String toString()
   {
      String string = getClass().getSimpleName() + ": value: " + getValueType() + ", derivative order: " + getDerivativeOrder() + ", first segment number: "
                    + firstSegmentNumber + ", constraint type: " + constraintType + ", first segment duration: " + firstSegmentDuration + ", omega: " + omega
                      + ", weight: " + weight + ".";
      for (int i = 0; i < firstSegmentContactPlaneHelpers.size(); i++)
      {
         string += "\nfirst segment contact " + i + ": " + firstSegmentContactPlaneHelpers.get(i);
      }
      for (int i = 0; i < secondSegmentContactPlaneHelpers.size(); i++)
      {
         string += "\nsecond segment contact " + i + ": " + secondSegmentContactPlaneHelpers.get(i);
      }
      return string;
   }
}
