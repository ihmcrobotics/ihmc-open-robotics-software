package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

/**
 * This command is designed to track a specified value for all the generalized contact accelerations along a basis vector over the course of the segment duration.
 * The exact cost is the integral of the squared difference of this generalized contact acceleration value from the objective value.
 */
public class RhoTrackingCommand implements MPCCommand<RhoTrackingCommand>
{
   private int commandId;
   /**
    * Defines the contact planes to be used in the force minimization.
    */
   private final List<MPCContactPlane> contactPlaneHelpers = new ArrayList<>();

   /**
    * Specifies the segment corresponding to these contacts.
    */
   private int segmentNumber;
   /**
    * Time constant used in the CoM function for this command.
    */
   private double omega;
   /**
    * Weight of this minimization command in the optimizer.
    */
   private double weight;
   /**
    * Duration of the specified segment.
    */
   private double segmentDuration;
   /**
    * Desired value for each generalized contact acceleration in the contact plane.
    */
   private double objectiveValue = 0.0;

   /**
    * Consumer for the computed cost to go on the output of the MPC function.
    */
   private DoubleConsumer costToGoConsumer;

   /**
    * @return command type for the MPC core.
    */
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.RHO_TRACKING;
   }

   /**
    * Resets the command
    */
   public void clear()
   {
      segmentNumber = -1;
      costToGoConsumer = null;
      segmentDuration = Double.NaN;
      objectiveValue = 0.0;
      contactPlaneHelpers.clear();
   }

   /**
    * Sets the weight for the value cost.
    */
   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   public void setSegmentDuration(double duration)
   {
      this.segmentDuration = duration;
   }

   public void setObjectiveValue(double objectiveValue)
   {
      this.objectiveValue = objectiveValue;
   }

   /**
    * Adds a contact whose force should be minimized
    */
   public void addContactPlaneHelper(MPCContactPlane contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
   }

   /**
    * Sets the segment corresponding to these contacts.
    */
   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
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

   public double getSegmentDuration()
   {
      return segmentDuration;
   }

   public double getObjectiveValue()
   {
      return objectiveValue;
   }

   public double getOmega()
   {
      return omega;
   }

   public int getNumberOfContacts()
   {
      return contactPlaneHelpers.size();
   }

   public MPCContactPlane getContactPlaneHelper(int i)
   {
      return contactPlaneHelpers.get(i);
   }

   public void setCostToGoConsumer(DoubleConsumer costToGoConsumer)
   {
      this.costToGoConsumer = costToGoConsumer;
   }

   public DoubleConsumer getCostToGoConsumer()
   {
      return costToGoConsumer;
   }

   public void setCostToGo(double costToGo)
   {
      if (costToGoConsumer != null)
         costToGoConsumer.accept(costToGo);
   }

   @Override
   public void set(RhoTrackingCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setOmega(other.getOmega());
      setWeight(other.getWeight());
      setCostToGoConsumer(other.getCostToGoConsumer());
      setSegmentDuration(other.getSegmentDuration());
      setObjectiveValue(other.getObjectiveValue());
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
      else if (object instanceof RhoTrackingCommand)
      {
         RhoTrackingCommand other = (RhoTrackingCommand) object;
         if (commandId != other.commandId)
            return false;
         if (segmentNumber != other.segmentNumber)
            return false;
         if (omega != other.omega)
            return false;
         if (weight != other.weight)
            return false;
         if (segmentDuration != other.segmentDuration)
            return false;
         if (contactPlaneHelpers.size() != other.contactPlaneHelpers.size())
            return false;
         if (objectiveValue != other.objectiveValue)
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
      String string = getClass().getSimpleName() + ": segment number: " + segmentNumber + ", omega: " + omega + ", weight: " + weight + ".";
      for (int i = 0; i < getNumberOfContacts(); i++)
      {
         string += "\ncontact " + i + ": " + contactPlaneHelpers.get(i);
      }
      return string;
   }
}
