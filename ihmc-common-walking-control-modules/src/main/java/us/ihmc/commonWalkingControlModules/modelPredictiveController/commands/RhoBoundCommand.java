package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;

import java.util.ArrayList;
import java.util.List;

/**
 * Specifies an objective value for each of the contact force vectors contained in the contact plane.
 *
 * Can be used to both specify a direct value, as well as upper and lower bounds for those forces.
 */
public class RhoBoundCommand implements MPCCommand<RhoBoundCommand>
{
   private int commandId;
   /**
    * Contact planes containing the generalized contact force vectors.
    */
   private final List<MPCContactPlane> contactPlanes = new ArrayList<>();

   private final TDoubleArrayList rhoValues = new TDoubleArrayList();

   private ConstraintType constraintType;

   /**
    * Segment numbers for those forces.
    */
   private int segmentNumber;
   /**
    * Time of this command.
    */
   private double segmentDuration;
   /**
    * Time constant for the motion function
    */
   private double omega;


   /**
    * Resets this command.
    */
   public void clear()
   {
      contactPlanes.clear();;
      rhoValues.reset();;
      segmentNumber = -1;
      segmentDuration = Double.NaN;
      constraintType = null;
   }

   /**
    * Adds a contact that contain the contact points that define the direction of the generalized contact forces.
    */
   public void addContactPlane(MPCContactPlane contactPlane, double rhoValue)
   {
      this.contactPlanes.add(contactPlane);
      this.rhoValues.add(rhoValue);
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
   public void setSegmentDuration(double segmentDuration)
   {
      this.segmentDuration = segmentDuration;
   }

   /**
    * Sets the time constant for the motion function
    */
   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public int getSegmentNumber()
   {
      return segmentNumber;
   }

   public double getSegmentDuration()
   {
      return segmentDuration;
   }

   public double getOmega()
   {
      return omega;
   }

   public int getNumberOfContacts()
   {
      return contactPlanes.size();
   }

   public MPCContactPlane getContactPlane(int i)
   {
      return contactPlanes.get(i);
   }

   public double getRhoValue(int i)
   {
      return rhoValues.get(i);
   }

   public void setConstraintType(ConstraintType constraintType)
   {
      this.constraintType = constraintType;
   }

   public ConstraintType getConstraintType()
   {
      return constraintType;
   }

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.RHO_BOUND;
   }

   @Override
   public void set(RhoBoundCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setSegmentDuration(other.getSegmentDuration());
      setOmega(other.getOmega());
      setConstraintType(other.getConstraintType());
      for (int i = 0; i < other.getNumberOfContacts(); i++)
         addContactPlane(other.getContactPlane(i), other.getRhoValue(i));
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
      else if (object instanceof RhoBoundCommand)
      {
         RhoBoundCommand other = (RhoBoundCommand) object;
         if (commandId != other.commandId)
            return false;
         if (segmentNumber != other.segmentNumber)
            return false;
         if (segmentDuration != other.segmentDuration)
            return false;
         if (omega != other.omega)
            return false;
         if (constraintType != other.constraintType)
            return false;
         if (getNumberOfContacts() != other.getNumberOfContacts())
            return false;
         for (int i = 0; i < getNumberOfContacts(); i++)
         {
            if (!contactPlanes.get(i).equals(other.contactPlanes.get(i)))
               return false;
            if (rhoValues.get(i) != other.getRhoValue(i))
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
      String string = getClass().getSimpleName() + ": segment number: " + segmentNumber + ", time of objective: " + segmentDuration + ", omega: " + omega;
      for (int i = 0; i < getNumberOfContacts(); i++)
      {
         string += "\ncontact " + i + " : " + contactPlanes.get(i);
      }
      return string;
   }
}
