package us.ihmc.sensorProcessing.stateEstimation;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.simulatedSensors.SensorMap;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class JointStateFullRobotModelUpdater extends AbstractControlFlowElement
{
   private final OneDoFJoint[] oneDoFJoints;

   private final Map<OneDoFJoint, ControlFlowInputPort<Double>> positionSensorInputPorts = new LinkedHashMap<OneDoFJoint, ControlFlowInputPort<Double>>();
   private final Map<OneDoFJoint, ControlFlowInputPort<Double>> velocitySensorInputPorts = new LinkedHashMap<OneDoFJoint, ControlFlowInputPort<Double>>();

   private final ControlFlowOutputPort<TwistCalculator> twistCalculatorOutputPort;
   private final ControlFlowOutputPort<SpatialAccelerationCalculator> spatialAccelerationCalculatorOutputPort;

   public JointStateFullRobotModelUpdater(ControlFlowGraph controlFlowGraph, SensorMap sensorMap, TwistCalculator twistCalculator,
           SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      this(controlFlowGraph, sensorMap.getJointPositionSensors(), sensorMap.getJointVelocitySensors(), twistCalculator, spatialAccelerationCalculator);
   }

   public JointStateFullRobotModelUpdater(ControlFlowGraph controlFlowGraph, Map<OneDoFJoint, ControlFlowOutputPort<Double>> positionSensorPorts,
           Map<OneDoFJoint, ControlFlowOutputPort<Double>> velocitySensorPorts, TwistCalculator twistCalculator,
           SpatialAccelerationCalculator spatialAccelerationCalculator)
   {
      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(twistCalculator.getRootBody());
      this.oneDoFJoints = ScrewTools.filterJoints(joints, OneDoFJoint.class);

      this.twistCalculatorOutputPort = createOutputPort();
      this.spatialAccelerationCalculatorOutputPort = createOutputPort();
      twistCalculatorOutputPort.setData(twistCalculator);
      spatialAccelerationCalculatorOutputPort.setData(spatialAccelerationCalculator);

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         if (positionSensorPorts.get(oneDoFJoint) == null)
         {
            throw new RuntimeException("positionSensorPorts.get(oneDoFJoint) == null. oneDoFJoint = " + oneDoFJoint);
         }
      }

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         ControlFlowOutputPort<Double> positionSensorOutputPort = positionSensorPorts.get(oneDoFJoint);
         ControlFlowInputPort<Double> positionSensorInputPort = createInputPort();

         positionSensorInputPorts.put(oneDoFJoint, positionSensorInputPort);
         controlFlowGraph.connectElements(positionSensorOutputPort, positionSensorInputPort);

         ControlFlowOutputPort<Double> velocitySensorOutputPort = velocitySensorPorts.get(oneDoFJoint);
         ControlFlowInputPort<Double> velocitySensorInputPort = createInputPort();

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

         ControlFlowInputPort<Double> positionSensorPort = positionSensorInputPorts.get(joint);
         Double positionSensorData = positionSensorPort.getData();

         joint.setQ(positionSensorData);
         joint.setQd(velocitySensorInputPorts.get(joint).getData());
         joint.setQdd(joint.getQddDesired());
      }

      // TODO: Does it make sense to do this yet if the orientation of the pelvis isn't known yet?
      TwistCalculator twistCalculator = twistCalculatorOutputPort.getData();
      SpatialAccelerationCalculator spatialAccelerationCalculator = spatialAccelerationCalculatorOutputPort.getData();

      twistCalculator.getRootBody().updateFramesRecursively();
      twistCalculator.compute();
      spatialAccelerationCalculator.compute();

      twistCalculatorOutputPort.setData(twistCalculator);
      spatialAccelerationCalculatorOutputPort.setData(spatialAccelerationCalculator);
   }

   public void waitUntilComputationIsDone()
   {
   }

   public ControlFlowOutputPort<TwistCalculator> getTwistCalculatorOutputPort()
   {
      return twistCalculatorOutputPort;
   }

   public ControlFlowOutputPort<SpatialAccelerationCalculator> getSpatialAccelerationCalculatorOutputPort()
   {
      return spatialAccelerationCalculatorOutputPort;
   }

}
