package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

/**
 * This command is designed to minimize the force exerted over the course of the segment duration. The exact cost is the integral of the squared force magnitude.
 * This can be thought of as minimizing the absolute work done.
 *
 * Whereas many other MPC algorithms provide some regularization by minimizing the CoM acceleration, this minimizes the force, so as to account for the force
 * that must be exerted to counter gravity.
 */
public class ForceObjectiveCommand implements MPCCommand<ForceObjectiveCommand>
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
    * Consumer for the computed cost to go on the output of the MPC function.
    */
   private DoubleConsumer costToGoConsumer;

   /**
    * @return command type for the MPC core.
    */
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.FORCE_VALUE;
   }

   /**
    * Resets the command
    */
   public void clear()
   {
      segmentNumber = -1;
      costToGoConsumer = null;
      contactPlaneHelpers.clear();
   }

   /**
    * Sets the weight for the value cost.
    */
   public void setWeight(double weight)
   {
      this.weight = weight;
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
   public void set(ForceObjectiveCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setOmega(other.getOmega());
      setWeight(other.getWeight());
      setCostToGoConsumer(other.getCostToGoConsumer());
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
      else if (object instanceof ForceObjectiveCommand)
      {
         ForceObjectiveCommand other = (ForceObjectiveCommand) object;
         if (commandId != other.commandId)
            return false;
         if (segmentNumber != other.segmentNumber)
            return false;
         if (omega != other.omega)
            return false;
         if (weight != other.weight)
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
      String string = getClass().getSimpleName() + ": segment number: " + segmentNumber + ", omega: " + omega + ", weight: " + weight + ".";
      for (int i = 0; i < getNumberOfContacts(); i++)
      {
         string += "\ncontact " + i + ": " + contactPlaneHelpers.get(i);
      }
      return string;
   }
}
