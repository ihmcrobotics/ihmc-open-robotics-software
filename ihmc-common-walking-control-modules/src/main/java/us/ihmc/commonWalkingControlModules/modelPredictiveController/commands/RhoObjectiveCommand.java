package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;

import java.util.ArrayList;
import java.util.List;

/**
 * Specifies an objective value for each of the contact force vectors contained in the contact plane.
 *
 * Can be used to both specify a direct value, as well as upper and lower bounds for those forces.
 */
public abstract class RhoObjectiveCommand implements MPCCommand<RhoObjectiveCommand>
{
   private int commandId;
   /**
    * Contact planes containing the generalized contact force vectors.
    */
   private final List<MPCContactPlane> contactPlaneHelpers = new ArrayList<>();

   /**
    * Segment numbers for those forces.
    */
   private int segmentNumber;
   /**
    * Time of this command.
    */
   private double timeOfObjective;
   /**
    * Time constant for the motion function
    */
   private double omega;

   /**
    * Constraint type to be used for achieving this command.
    */
   private ConstraintType constraintType;

   /**
    * Singular scalar value to be used as the objective for all the generalized contact force values.
    */
   private double objective = Double.NaN;
   /**
    * Vector of scalar values objectives for each generalized contact force.
    */
   private final DMatrixRMaj objectiveVector = new DMatrixRMaj(0, 0);
   /**
    * Whether or not to use the scalar objective for every contact force, or to use specific objective values.
    */
   private boolean useScalarObjective = true;

   /**
    * Resets this command.
    */
   public void clear()
   {
      contactPlaneHelpers.clear();
      objective = 0.0;
      objectiveVector.reshape(0, 0);
      segmentNumber = -1;
      timeOfObjective = Double.NaN;
   }

   public abstract int getDerivativeOrder();

   /**
    * Adds a contact that contain the contact points that define the direction of the generalized contact forces.
    */
   public void addContactPlaneHelper(MPCContactPlane contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
   }

   /**
    * Set the constraint type for this objective (lower ineq, upper ineq, objective, etc.)
    */
   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   /**
    * Sets the motion segment that defines the variables for this command
    */
   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   /**
    * Sets the time in the motion segment for this command.
    */
   public void setTimeOfObjective(double timeOfObjective)
   {
      this.timeOfObjective = timeOfObjective;
   }

   /**
    * Sets the time constant for the motion function
    */
   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   /**
    * Sets the scalar objective value for all the generalized contact forces.
    */
   public void setScalarObjective(double objective)
   {
      this.objective = objective;
   }

   /**
    * Sets the objective values for the generalized contact forces.
    */
   public void setObjectiveVector(DMatrix objective)
   {
      this.objectiveVector.set(objective);
   }

   /**
    * Sets whether or not to use the same objective value for every generalized contact force.
    */
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

   public MPCContactPlane getContactPlaneHelper(int i)
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

   @Override
   public void set(RhoObjectiveCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setTimeOfObjective(other.getTimeOfObjective());
      setOmega(other.getOmega());
      setConstraintType(other.getConstraintType());
      setObjectiveVector(other.getObjectiveVector());
      setUseScalarObjective(other.getUseScalarObjective());
      setScalarObjective(other.getScalarObjective());
      for (int i = 0; i < other.getNumberOfContacts(); i++)
         addContactPlaneHelper(other.getContactPlaneHelper(i));
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
      else if (object instanceof RhoObjectiveCommand)
      {
         RhoObjectiveCommand other = (RhoObjectiveCommand) object;
         if (commandId != other.commandId)
            return false;
         if (segmentNumber != other.segmentNumber)
            return false;
         if (timeOfObjective != other.timeOfObjective)
            return false;
         if (omega != other.omega)
            return false;
         if (useScalarObjective != other.useScalarObjective)
            return false;
         if (useScalarObjective)
         {
            if (objective != other.objective)
               return false;
         }
         else if (objectiveVector != other.objectiveVector)
            return false;
         if (contactPlaneHelpers.size() != other.contactPlaneHelpers.size())
            return false;
         for (int i = 0; i < contactPlaneHelpers.size(); i++)
         {
            if (!contactPlaneHelpers.get(i).equals(other.contactPlaneHelpers.get(i)))
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
      String string = getClass().getSimpleName() + ": segment number: " + segmentNumber + ", time of objective: " + timeOfObjective + ", omega: " + omega
                      + ", constraint type: " + constraintType;
      if (useScalarObjective)
         string += "objective value: " + objective;
      else
         string += "objective vector: " + objectiveVector + ".";
      for (int i = 0; i < contactPlaneHelpers.size(); i++)
      {
         string += "\ncontact " + i + ": " + contactPlaneHelpers.get(i);
      }
      return string;
   }
}
