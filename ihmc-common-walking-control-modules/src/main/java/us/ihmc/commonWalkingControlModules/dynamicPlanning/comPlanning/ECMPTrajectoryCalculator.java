package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory3DReadOnly;

import java.util.List;

public class ECMPTrajectoryCalculator
{
   private final RecyclingArrayList<SettableContactStateProvider> contactStateProviders = new RecyclingArrayList<>(SettableContactStateProvider::new);
   private final double mass;
   private final double gravity;
   private final double weight;

   public ECMPTrajectoryCalculator(double mass, double gravity)
   {
      this.mass = mass;
      this.gravity = Math.abs(gravity);

      weight = mass * gravity;
   }

   private final FramePoint3D ecmpPosition = new FramePoint3D();
   private final FrameVector3D ecmpVelocity = new FrameVector3D();

   // TODO in its current form, this assumes that the desired net angular momentum rate is zero.
   public void computeECMPTrajectory(List<? extends ContactStateProvider> copTrajectories, List<Trajectory3DReadOnly> internalAngularMomentumTrajectories)
   {
      contactStateProviders.clear();
      for (int i = 0; i < copTrajectories.size(); i++)
      {
         contactStateProviders.add().set(copTrajectories.get(i));
      }

      for (int i = 0; i < Math.min(copTrajectories.size(), internalAngularMomentumTrajectories.size()); i++)
      {
         ContactStateProvider copTrajectory = copTrajectories.get(i);
         SettableContactStateProvider eCMPTrajectory = contactStateProviders.get(i);
         Trajectory3DReadOnly angularMomentumTrajectory = internalAngularMomentumTrajectories.get(i);

         if (copTrajectory.getTimeInterval().epsilonEquals(angularMomentumTrajectory, 1e-4))
            throw new IllegalArgumentException("The time intervals are not proper.");

         double startTime = angularMomentumTrajectory.getStartTime();
         angularMomentumTrajectory.compute(startTime);
         Vector3DReadOnly initialAngularRate = angularMomentumTrajectory.getVelocity();
         Vector3DReadOnly initialAngularAcceleration = angularMomentumTrajectory.getAcceleration();

         // These are scaled by a negative one because we want the total angular momentum to be zero
         ecmpPosition.setX(initialAngularRate.getY());
         ecmpPosition.setY(-initialAngularRate.getX());
         ecmpPosition.scale(-1.0 / weight);
         ecmpPosition.add(copTrajectory.getECMPStartPosition());

         ecmpVelocity.setX(initialAngularAcceleration.getY());
         ecmpVelocity.setY(-initialAngularAcceleration.getX());
         ecmpVelocity.scale(-1.0 / weight);
         ecmpVelocity.add(copTrajectory.getECMPStartVelocity());

         eCMPTrajectory.setStartCopPosition(ecmpPosition);
         eCMPTrajectory.setStartCopVelocity(ecmpVelocity);


         double endTime = angularMomentumTrajectory.getEndTime();

         angularMomentumTrajectory.compute(endTime);
         Vector3DReadOnly finalAngularRate = angularMomentumTrajectory.getVelocity();
         Vector3DReadOnly finalAngularAcceleration = angularMomentumTrajectory.getAcceleration();

         ecmpPosition.setX(finalAngularRate.getY());
         ecmpPosition.setY(-finalAngularRate.getX());
         ecmpPosition.scale(-1.0 / weight);
         ecmpPosition.add(copTrajectory.getECMPEndPosition());

         ecmpVelocity.setX(finalAngularAcceleration.getY());
         ecmpVelocity.setY(-finalAngularAcceleration.getX());
         ecmpVelocity.scale(-1.0 / weight);
         ecmpVelocity.add(copTrajectory.getECMPEndVelocity());

         eCMPTrajectory.setEndCopPosition(ecmpPosition);
         eCMPTrajectory.setEndCopVelocity(ecmpVelocity);
      }
   }

   public RecyclingArrayList<SettableContactStateProvider> getContactStateProviders()
   {
      return contactStateProviders;
   }
}
