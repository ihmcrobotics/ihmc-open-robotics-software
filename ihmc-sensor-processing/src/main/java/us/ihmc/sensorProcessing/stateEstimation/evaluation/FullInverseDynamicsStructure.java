package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullRobotModel;

public class FullInverseDynamicsStructure
{
   private final RigidBodyBasics estimationLink;
   private final RigidBodyBasics elevator;
   private final FloatingJointBasics rootJoint;

   // TODO: What's a good name for this?
   public FullInverseDynamicsStructure(RigidBodyBasics elevator, RigidBodyBasics estimationLink, FloatingJointBasics rootInverseDynamicsJoint)
   {
      this.elevator = elevator;
      this.rootJoint = rootInverseDynamicsJoint;
      this.estimationLink = estimationLink;
   }

   public FloatingJointBasics getRootJoint()
   {
      return rootJoint;
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

   public static FullInverseDynamicsStructure createInverseDynamicStructure(FullRobotModel fullRobotModel)
   {
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      FloatingJointBasics rootInverseDynamicsJoint = fullRobotModel.getRootJoint();
      RigidBodyBasics estimationLink = fullRobotModel.getRootBody();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      return inverseDynamicsStructure;
   }
}
