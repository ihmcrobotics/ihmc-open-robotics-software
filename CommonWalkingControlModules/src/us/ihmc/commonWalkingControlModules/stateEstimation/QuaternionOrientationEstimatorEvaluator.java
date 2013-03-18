package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.Random;

import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.signalCorruption.GaussianOrientationCorruptor;
import us.ihmc.sensorProcessing.simulatedSensors.SimulatedOrientationSensor;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.ScrewTestTools;
import us.ihmc.utilities.screwTheory.ScrewTestTools.RandomFloatingChain;
import us.ihmc.utilities.screwTheory.SixDoFJoint;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class QuaternionOrientationEstimatorEvaluator
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

   private final double orientationStandardDeviation = 1e-3;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final RandomFloatingChain randomFloatingChain;

   public QuaternionOrientationEstimatorEvaluator()
   {
      Random random = new Random(1235125L);
      randomFloatingChain = new RandomFloatingChain(random, new Vector3d[] {X, Y, Z});

      SixDoFJoint rootJoint = randomFloatingChain.getRootJoint();
      ScrewTestTools.setRandomPositionAndOrientation(rootJoint, random);
      ScrewTestTools.setRandomVelocity(rootJoint, random);
      ScrewTestTools.setRandomPositions(randomFloatingChain.getRevoluteJoints(), random);
      ScrewTestTools.setRandomVelocities(randomFloatingChain.getRevoluteJoints(), random);

      SensorConfiguration<FrameOrientation> orientationSensorConfiguration = new SensorConfiguration<FrameOrientation>();

      SimulatedOrientationSensor sensor = new SimulatedOrientationSensor(rootJoint.getFrameAfterJoint());
      GaussianOrientationCorruptor orientationCorruptor = new GaussianOrientationCorruptor("gaussianOrientation", 12345L, registry);
      orientationCorruptor.setStandardDeviation(orientationStandardDeviation);
      sensor.addSignalCorruptor(orientationCorruptor);

      SensorConfiguration<FrameVector> angularVelocitySensorConfiguration = new SensorConfiguration<FrameVector>();

      ReferenceFrame estimationFrame = rootJoint.getFrameAfterJoint();

      double controlDT = orientationStandardDeviation;
      QuaternionOrientationEstimator estimator = new QuaternionOrientationEstimator(name, orientationSensorConfiguration, angularVelocitySensorConfiguration,
                                                    estimationFrame, controlDT, registry);

      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.addYoVariableRegistry(registry);

      // TODO: tick and update
   }

   public static void main(String[] args)
   {
      new QuaternionOrientationEstimatorEvaluator();
   }
}
