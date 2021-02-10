package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.CoMTrajectoryModelPredictiveController;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneHelper;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

/**
 * Tracking command a nominal VRP trajectory. Specifies a nominal VRP trajectory for the motion function to try and achieve for the segment
 * {@link #getSegmentNumber()} over a duration {@link #getSegmentDuration()}. This motion function always takes the form of a linear function that goes from
 * {@link #getStartVRP()} to {@link #getEndVRP()}.
 *
 * TODO: this tracking function should be improved to allow cubic VRP trajectories, not just linear ones.
 *
 * This tracking command can be formulated into a closed-form quadratic cost for the optimizer, and always represents an
 * {@link us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType#OBJECTIVE}
 */
public class VRPTrackingCommand implements MPCCommand<VRPTrackingCommand>
{
   /**
    * Contact planes to be used to track the VRP
    */
   private final List<ContactPlaneHelper> contactPlaneHelpers = new ArrayList<>();

   /**
    * Segment that is trying to track the VRP
    */
   private int segmentNumber;
   /**
    * Duration of the segment that is trying to track the VRP
    */
   private double segmentDuration;
   /**
    * Time constant used in the motion function.
    */
   private double omega;
   /**
    *  weight to scale the cost of the VRP tracking objective.
    */
   private double weight = CoMTrajectoryModelPredictiveController.vrpTrackingWeight;

   /**
    * Desired VRP value at the beginning of the segment.
    */
   private final FramePoint3D startVRP = new FramePoint3D();
   /**
    * Desired VRP value at the end of the segment.
    */
   private final FramePoint3D endVRP = new FramePoint3D();

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
      segmentDuration = Double.NaN;
      segmentNumber = -1;
      startVRP.setToNaN();
      endVRP.setToNaN();
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

   public FramePoint3DReadOnly getStartVRP()
   {
      return startVRP;
   }

   public FramePoint3DReadOnly getEndVRP()
   {
      return endVRP;
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
   public void addContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
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
   public void setSegmentDuration(double segmentDuration)
   {
      this.segmentDuration = segmentDuration;
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
      return contactPlaneHelpers.size();
   }

   public ContactPlaneHelper getContactPlaneHelper(int i)
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

   public void setCostToGo(double costToGo)
   {
      if (costToGoConsumer != null)
         costToGoConsumer.accept(costToGo);
   }
}
