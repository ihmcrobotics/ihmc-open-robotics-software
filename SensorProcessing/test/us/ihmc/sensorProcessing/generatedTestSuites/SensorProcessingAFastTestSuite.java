package us.ihmc.sensorProcessing.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.controlFlow.ControlFlowTest.class,
   us.ihmc.sensorProcessing.controlFlowPorts.YoFramePointControlFlowOutputPortTest.class,
   us.ihmc.sensorProcessing.controlFlowPorts.YoFrameQuaternionControlFlowOutputPortTest.class,
   us.ihmc.sensorProcessing.controlFlowPorts.YoFrameVectorControlFlowOutputPortTest.class,
   us.ihmc.sensorProcessing.diagnostic.DelayEstimatorBetweenTwoSignalsTest.class,
   us.ihmc.sensorProcessing.encoder.SimulatedEncoderTest.class,
   us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.GroundOnlyQuadTreeTest.class,
   us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMapTest.class,
   us.ihmc.sensorProcessing.pointClouds.shape.ExpectationMaximizationFitterTest.class,
   us.ihmc.sensorProcessing.pointClouds.shape.FitPlaneWeighted3DTest.class,
   us.ihmc.sensorProcessing.sensorProcessors.ElasticityCompensatorYoVariableTest.class,
   us.ihmc.sensorProcessing.SimulatedIMURawSensorReader2Test.class,
   us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGeneratorTest.class,
   us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.AngularVelocityMeasurementModelElementTest.class,
   us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.LinearAccelerationMeasurementModelElementTest.class,
   us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.OrientationMeasurementModelElementTest.class,
   us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.PointPositionMeasurementModelElementTest.class,
   us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.PointVelocityMeasurementModelElementTest.class,
   us.ihmc.sensorProcessing.stateEstimation.OrientationAndPositionFullRobotModelUpdaterTest.class,
   us.ihmc.sensorProcessing.stateEstimation.YoPointPositionDataObjectListOutputPortTest.class
})

public class SensorProcessingAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
