package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;

import java.util.ArrayList;
import java.util.List;

public abstract class CoMContinuityCommand implements MPCCommand<CoMContinuityCommand>
{
   private final List<ContactPlaneHelper> firstSegmentContactPlaneHelpers = new ArrayList<>();
   private final List<ContactPlaneHelper> secondSegmentContactPlaneHelpers = new ArrayList<>();

   private int firstSegmentNumber;
   private double firstSegmentDuration;
   private double omega;
   private double weight = CoMTrajectoryModelPredictiveController.MEDIUM_WEIGHT;

   public MPCCommandType getCommandType()
   {
      return MPCCommandType.CONTINUITY;
   }

   public abstract int getDerivativeOrder();

   public void clear()
   {
      firstSegmentContactPlaneHelpers.clear();
      secondSegmentContactPlaneHelpers.clear();
   }

   public void addFirstSegmentContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      firstSegmentContactPlaneHelpers.add(contactPlaneHelper);
   }

   public void addSecondSegmentContactPlaneHelper(ContactPlaneHelper contactPlaneHelper)
   {
      secondSegmentContactPlaneHelpers.add(contactPlaneHelper);
   }

   public void setFirstSegmentNumber(int firstSegmentNumber)
   {
      this.firstSegmentNumber = firstSegmentNumber;
   }

   public void setFirstSegmentDuration(double firstSegmentDuration)
   {
      this.firstSegmentDuration = firstSegmentDuration;
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

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
