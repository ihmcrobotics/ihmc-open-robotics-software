package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommandType;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class VRPTrackingCommand implements MPCCommand<VRPTrackingCommand>
{
   private final FramePoint3D objective = new FramePoint3D();
   private final List<ContactPlaneHelper> contactPlaneHelpers = new ArrayList<>();

   private int segmentNumber;
   private double segmentDuration;
   private double omega;
   private double weight = CoMTrajectoryModelPredictiveController.vrpTrackingWeight;

   private final FramePoint3D startVRP = new FramePoint3D();
   private final FramePoint3D endVRP = new FramePoint3D();

   public void clear()
   {
      contactPlaneHelpers.clear();
   }

   public void setStartVRP(FramePoint3DReadOnly startVRP)
   {
      this.startVRP.set(startVRP);
   }

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

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.VRP_TRACKING;
   }
}
