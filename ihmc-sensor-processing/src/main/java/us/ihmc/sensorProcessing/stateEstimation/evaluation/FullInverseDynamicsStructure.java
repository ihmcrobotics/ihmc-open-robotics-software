package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;

public class FullInverseDynamicsStructure
{
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBodyBasics estimationLink;
   private final RigidBodyBasics elevator;
   private final FloatingInverseDynamicsJoint rootJoint;

   // TODO: What's a good name for this?
   public FullInverseDynamicsStructure(RigidBodyBasics elevator, RigidBodyBasics estimationLink, FloatingInverseDynamicsJoint rootInverseDynamicsJoint)
   {
      this.elevator = elevator;
      this.rootJoint = rootInverseDynamicsJoint;

      spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, 0.0);

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

   public RigidBodyBasics getEstimationLink()
   {
      return estimationLink;
   }

   public ReferenceFrame getEstimationFrame()
   {
      return estimationLink.getParentJoint().getFrameAfterJoint();
   }

   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   public void updateInternalState()
   {
      spatialAccelerationCalculator.compute();
   }

   public static FullInverseDynamicsStructure createInverseDynamicStructure(FullRobotModel fullRobotModel)
   {
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      FloatingInverseDynamicsJoint rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBodyBasics estimationLink = fullRobotModel.getRootBody();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      return inverseDynamicsStructure;
   }
}
