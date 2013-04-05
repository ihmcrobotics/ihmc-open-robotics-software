package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.HashMap;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class JointStateFullRobotModelUpdater implements Runnable
{
   private final OneDoFJoint[] oneDoFJoints;
   private final HashMap<OneDoFJoint, ControlFlowOutputPort<Double>> positionSensorPorts;
   private final HashMap<OneDoFJoint, ControlFlowOutputPort<Double>> velocitySensorPorts;
   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

   public JointStateFullRobotModelUpdater(HashMap<OneDoFJoint, ControlFlowOutputPort<Double>> positionSensorPorts,
         HashMap<OneDoFJoint, ControlFlowOutputPort<Double>> velocitySensorPorts, TwistCalculator twistCalculator,
         SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(twistCalculator.getRootBody());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);
      this.positionSensorPorts = positionSensorPorts;
      this.velocitySensorPorts = velocitySensorPorts;
      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;
   }

   public void run()
   {
      for (OneDoFJoint joint : oneDoFJoints)
      {
         joint.setQ(positionSensorPorts.get(joint).getData());
         joint.setQ(velocitySensorPorts.get(joint).getData());
         joint.setQdd(joint.getQddDesired());
      }

      twistCalculator.getRootBody().updateFramesRecursively();
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
   }
}
