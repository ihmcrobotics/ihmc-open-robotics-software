package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class FullInverseDynamicsStructure
{
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;
   private final RigidBody estimationLink;
   private final RigidBody elevator;
   private final SixDoFJoint rootJoint;

   // TODO: What's a good name for this?
   public FullInverseDynamicsStructure(RigidBody elevator, RigidBody estimationLink, SixDoFJoint rootInverseDynamicsJoint)
   {
      this.elevator = elevator;
      this.rootJoint = rootInverseDynamicsJoint;

      twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), elevator);
      spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, 0.0, false);

      this.estimationLink = estimationLink;
   }
   
   public SixDoFJoint getRootJoint()
   {
      return rootJoint;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
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
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
   }

   public static FullInverseDynamicsStructure createInverseDynamicStructure(SDFFullRobotModel sdfFullRobotModel)
   {
      RigidBody elevator = sdfFullRobotModel.getElevator();
      SixDoFJoint rootInverseDynamicsJoint = sdfFullRobotModel.getRootJoint();
      RigidBody estimationLink = sdfFullRobotModel.getPelvis();

      FullInverseDynamicsStructure inverseDynamicsStructure = new FullInverseDynamicsStructure(elevator, estimationLink, rootInverseDynamicsJoint);

      return inverseDynamicsStructure;
   }
}
