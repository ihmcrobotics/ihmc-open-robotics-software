package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCParameters;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

/**
 * Tracking command a nominal VRP trajectory. Specifies a nominal VRP trajectory for the motion function to try and achieve for the segment
 * {@link #getSegmentNumber()} over a duration. This motion function always takes the form of a linear function that goes from
 * {@link #getStartVRP()} to {@link #getEndVRP()}.
 *
 * This tracking command can be formulated into a closed-form quadratic cost for the optimizer, and always represents an
 * {@link us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType#OBJECTIVE}
 */
public class VRPTrackingCommand implements MPCCommand<VRPTrackingCommand>
{
   private int commandId;

   /**
    * Contact planes to be used to track the VRP
    */
   private final List<MPCContactPlane> contactPlaneHelpers = new ArrayList<>();

   /**
    * Segment that is trying to track the VRP
    */
   private int segmentNumber;
   /**
    * Start time of the segment that is trying to track the VRP
    */
   private double startTime;
   /**
    * End time of the segment that is trying to track the VRP
    */
   private double endTime;
   /**
    * Time constant used in the motion function.
    */
   private double omega;
   /**
    *  weight to scale the cost of the VRP tracking objective.
    */
   private double weight = MPCParameters.defaultVrpTrackingWeight;

   /**
    * Desired VRP value at the beginning of the segment.
    */
   private final FramePoint3D startVRP = new FramePoint3D();

   /**
    * Desired VRP value at the end of the segment.
    */
   private final FramePoint3D endVRP = new FramePoint3D();

   /**
    * Desired VRP velocity value at the beginning of the segment.
    */
   private final FrameVector3D startVRPVelocity = new FrameVector3D();

   /**
    * Desired VRP velocity value at the end of the segment.
    */
   private final FrameVector3D endVRPVelocity = new FrameVector3D();

   {
      startVRPVelocity.setToNaN();
      endVRPVelocity.setToNaN();
   }

   /**
    * Consumer for the computed cost to go on the output of the MPC function.
    */
   private DoubleConsumer costToGoConsumer;

   /**
    * Resets this objective.
    */
   public void clear()
   {
      costToGoConsumer = null;
      contactPlaneHelpers.clear();
      startTime = Double.NaN;
      endTime = Double.NaN;
      segmentNumber = -1;
      startVRP.setToNaN();
      endVRP.setToNaN();
      startVRPVelocity.setToNaN();
      endVRPVelocity.setToNaN();
   }

   /**
    * Sets the desired value of the VRP at the beginning of the segment.
    */
   public void setStartVRP(FramePoint3DReadOnly startVRP)
   {
      this.startVRP.set(startVRP);
   }

   /**
    * Sets the desired value of the VRP at the end of the segment.
    */
   public void setEndVRP(FramePoint3DReadOnly endVRP)
   {
      this.endVRP.set(endVRP);
   }

   public void setStartVRPVelocity(FrameVector3DReadOnly startVRPVelocity)
   {
      this.startVRPVelocity.set(startVRPVelocity);
   }

   public void setEndVRPVelocity(FrameVector3DReadOnly endVRPVelocity)
   {
      this.endVRPVelocity.set(endVRPVelocity);
   }

   public FramePoint3DReadOnly getStartVRP()
   {
      return startVRP;
   }

   public FramePoint3DReadOnly getEndVRP()
   {
      return endVRP;
   }

   public FrameVector3DReadOnly getStartVRPVelocity()
   {
      return startVRPVelocity;
   }

   public FrameVector3DReadOnly getEndVRPVelocity()
   {
      return endVRPVelocity;
   }

   /**
    * Sets the weight to scale the cost of the VRP tracking objective.
    */
   public void setWeight(double weight)
   {
      this.weight = weight;
   }

   /**
    * Adds a contact that the MPC can use to try and track the desired VRP
    */
   public void addContactPlaneHelper(MPCContactPlane contactPlaneHelper)
   {
      this.contactPlaneHelpers.add(contactPlaneHelper);
   }

   /**
    * Sets the segment number to be used to track the VRP trajectory.
    */
   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   /**
    * Sets the duration of the segment to track the VRP over.
    */
   public void setTimeInterval(double startTime, double endTime)
   {
      this.startTime = startTime;
      this.endTime = endTime;
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

   public double getStartTime()
   {
      return startTime;
   }

   public double getEndTime()
   {
      return endTime;
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

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.VRP_TRACKING;
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
   public void set(VRPTrackingCommand other)
   {
      clear();
      setCommandId(other.getCommandId());
      setSegmentNumber(other.getSegmentNumber());
      setTimeInterval(other.getStartTime(), other.getEndTime());
      setOmega(other.getOmega());
      setWeight(other.getWeight());
      setStartVRP(other.getStartVRP());
      setEndVRP(other.getEndVRP());
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
      else if (object instanceof VRPTrackingCommand)
      {
         VRPTrackingCommand other = (VRPTrackingCommand) object;
         if (commandId != other.commandId)
            return false;
         if (segmentNumber != other.segmentNumber)
            return false;
         if (startTime != other.startTime)
            return false;
         if (endTime != other.endTime)
            return false;
         if (omega != other.omega)
            return false;
         if (weight != other.weight)
            return false;
         if (!startVRP.equals(other.startVRP))
            return false;
         if (!endVRP.equals(other.endVRP))
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
      String string = getClass().getSimpleName() + ": segment number: " + segmentNumber + ", segment start time: " + startTime + ", segment end time: " + endTime
                      +", omega: " + omega + ", weight: " + weight + ", start vrp: " + startVRP + ", end vrp: " + endVRP + ".";
      for (int i = 0; i < contactPlaneHelpers.size(); i++)
      {
         string += "\ncontact " + i + ": " + contactPlaneHelpers.get(i);
      }
      return string;
   }
}
