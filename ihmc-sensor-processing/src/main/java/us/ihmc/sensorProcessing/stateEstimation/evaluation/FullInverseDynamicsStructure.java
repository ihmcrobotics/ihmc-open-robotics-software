package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;

public class FullInverseDynamicsStructure
{
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBody estimationLink;
   private final RigidBody elevator;
   private final FloatingInverseDynamicsJoint rootJoint;

   // TODO: What's a good name for this?
   public FullInverseDynamicsStructure(RigidBody elevator, RigidBody estimationLink, FloatingInverseDynamicsJoint rootInverseDynamicsJoint)
   {
      this.elevator = elevator;
      this.rootJoint = rootInverseDynamicsJoint;

      spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, 0.0, false);

      this.estimationLink = estimationLink;
   }

   public FloatingInverseDynamicsJoint getRootJoint()
   {
      return rootJoint;
   }

   public SpatialAccelerationCalculator getSpatialAccelerationCalculator()
   {
      return spatialAccelerationCalculator;
   }

   public RigidBody getEstimationLink()
   {
      return estimationLink;
   }

   public ReferenceFrame getEstimationFrame()
   {
      return estimationLink.getParentJoint().getFrameAfterJoint();
   }

   public RigidBody getElevator()
   {
      return elevator;
   }

   public void updateInternalState()
   {
      spatialAccelerationCalculator.compute();
   }

   public static FullInverseDynamicsStructure createInverseDynamicStructure(FullRobotModel fullRobotModel)
   {
      RigidBody elevator = fullRobotModel.getElevator();
      FloatingInverseDynamicsJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBody estimationLink = fullRobotModel.getPelvis();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      return inverseDynamicsStructure;
   }
}
