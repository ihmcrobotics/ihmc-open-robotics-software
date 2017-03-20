package us.ihmc.sensorProcessing.stateEstimation;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.simulatedSensors.JointAndIMUSensorMap;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class JointStateFullRobotModelUpdater extends AbstractControlFlowElement
{
   private final OneDoFJoint[] oneDoFJoints;

   private final Map<OneDoFJoint, ControlFlowInputPort<double[]>> positionSensorInputPorts = new LinkedHashMap<OneDoFJoint, ControlFlowInputPort<double[]>>();
   private final Map<OneDoFJoint, ControlFlowInputPort<double[]>> velocitySensorInputPorts = new LinkedHashMap<OneDoFJoint, ControlFlowInputPort<double[]>>();

   private final ControlFlowOutputPort<FullInverseDynamicsStructure> inverseDynamicsStructureOutputPort;

   public JointStateFullRobotModelUpdater(ControlFlowGraph controlFlowGraph, JointAndIMUSensorMap sensorMap,
           FullInverseDynamicsStructure inverseDynamicsStructure)
   {
      this(controlFlowGraph, sensorMap.getJointPositionSensors(), sensorMap.getJointVelocitySensors(), inverseDynamicsStructure);
   }

   public JointStateFullRobotModelUpdater(ControlFlowGraph controlFlowGraph, Map<OneDoFJoint, ControlFlowOutputPort<double[]>> jointPositionSensors,
           Map<OneDoFJoint, ControlFlowOutputPort<double[]>> jointVelocitySensors, FullInverseDynamicsStructure inverseDynamicsStructure)
   {
      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(inverseDynamicsStructure.getRootJoint().getSuccessor());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);

      this.inverseDynamicsStructureOutputPort = createOutputPort("inverseDynamicsStructureOutputPort");
      inverseDynamicsStructureOutputPort.setData(inverseDynamicsStructure);

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         if (jointPositionSensors.get(oneDoFJoint) == null)
         {
            throw new RuntimeException("positionSensorPorts.get(oneDoFJoint) == null. oneDoFJoint = " + oneDoFJoint);
         }
      }

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         ControlFlowOutputPort<double[]> positionSensorOutputPort = jointPositionSensors.get(oneDoFJoint);
         ControlFlowInputPort<double[]> positionSensorInputPort = createInputPort("positionSensorInputPort");

         positionSensorInputPorts.put(oneDoFJoint, positionSensorInputPort);
         controlFlowGraph.connectElements(positionSensorOutputPort, positionSensorInputPort);

         ControlFlowOutputPort<double[]> velocitySensorOutputPort = jointVelocitySensors.get(oneDoFJoint);
         ControlFlowInputPort<double[]> velocitySensorInputPort = createInputPort("velocitySensorInputPort");

         velocitySensorInputPorts.put(oneDoFJoint, velocitySensorInputPort);
         controlFlowGraph.connectElements(velocitySensorOutputPort, velocitySensorInputPort);
      }
   }

   public void startComputation()
   {
      for (OneDoFJoint joint : oneDoFJoints)
      {
         if (joint == null)
            throw new RuntimeException();

         ControlFlowInputPort<double[]> positionSensorPort = positionSensorInputPorts.get(joint);
         ControlFlowInputPort<double[]> velocitySensorPort = velocitySensorInputPorts.get(joint);

         double positionSensorData = positionSensorPort.getData()[0];
         double velocitySensorData = velocitySensorPort.getData()[0];

         joint.setQ(positionSensorData);
         joint.setQd(velocitySensorData);
         joint.setQdd(joint.getQddDesired());
      }

      // TODO: Does it make sense to do this yet if the orientation of the pelvis isn't known yet?
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureOutputPort.getData();

      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();

      twistCalculator.getRootBody().updateFramesRecursively();
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();

      inverseDynamicsStructureOutputPort.setData(inverseDynamicsStructure);
   }

   public void waitUntilComputationIsDone()
   {
   }

   public ControlFlowOutputPort<FullInverseDynamicsStructure> getInverseDynamicsStructureOutputPort()
   {
      return inverseDynamicsStructureOutputPort;
   }

   public void initialize()
   {
      startComputation();
      waitUntilComputationIsDone();
   }
}
