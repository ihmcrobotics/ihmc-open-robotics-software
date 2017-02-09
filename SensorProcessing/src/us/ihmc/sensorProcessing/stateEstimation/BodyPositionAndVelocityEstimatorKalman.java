package us.ihmc.sensorProcessing.stateEstimation;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.kalman.KalmanFilter;
import us.ihmc.kalman.YoKalmanFilter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.SensorProcessor;
import us.ihmc.sensorProcessing.sensors.ProcessedBodyPositionSensorsWriteOnlyInterface;
import us.ihmc.sensorProcessing.sensors.ProcessedIMUSensorsReadOnlyInterface;
import us.ihmc.sensorProcessing.sensors.ProcessedTimeSensorsReadOnlyInterface;

public class BodyPositionAndVelocityEstimatorKalman implements BodyPositionAndVelocityEstimator
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final ProcessedIMUSensorsReadOnlyInterface processedIMUSensors;
   private final ProcessedBodyPositionSensorsWriteOnlyInterface processedBodyPositionSensors;
   private final ProcessedTimeSensorsReadOnlyInterface processedTimeSensorsReadOnlyInterface;
   private final SensorProcessor bodyReferenceFrameUpdater;

   private final LinkedHashMap<BodyPositionEstimator, Integer> bodyPositionEstimatorIndices = new LinkedHashMap<BodyPositionEstimator, Integer>();
   private final LinkedHashMap<BodyVelocityEstimator, Integer> bodyVelocityEstimatorIndices = new LinkedHashMap<BodyVelocityEstimator, Integer>();
   private final BodyPositionEstimator bodyPositionEstimatorVicon;

   private final EnumMap<Direction, KalmanFilter> kalmanFilters = new EnumMap<Direction, KalmanFilter>(Direction.class);

   private final EnumMap<Direction, DenseMatrix64F> processCovariances = new EnumMap<Direction, DenseMatrix64F>(Direction.class);
   private final EnumMap<Direction, DenseMatrix64F> measurementCovariances = new EnumMap<Direction, DenseMatrix64F>(Direction.class);
   private final EnumMap<Direction, DenseMatrix64F> inputs = new EnumMap<Direction, DenseMatrix64F>(Direction.class);
   private final EnumMap<Direction, DenseMatrix64F> measurements = new EnumMap<Direction, DenseMatrix64F>(Direction.class);

   private final DoubleYoVariable accelerationCovariance = new DoubleYoVariable("accelerationCovariance", registry);
   private final DoubleYoVariable velocityCovariance = new DoubleYoVariable("velocityCovariance", registry);

   private final FramePoint bodyPosition = new FramePoint(world);
   private final FrameVector bodyVelocity = new FrameVector(world);
   private final FramePoint initialPositionIfNotUsingVicon;

   private final int positionIndex = 0;
   private final int velocityIndex = 1;
   private final int nStates = 2;
   private final int nInputs = 1;
   private int nMeasurements;
   
   private final int bodyIMUIndex;
   private final boolean configureEveryTick;

   public BodyPositionAndVelocityEstimatorKalman(ProcessedIMUSensorsReadOnlyInterface processedIMUSensors,
           ProcessedBodyPositionSensorsWriteOnlyInterface processedBodyPositionSensors, ProcessedTimeSensorsReadOnlyInterface processedTimeSensorsReadOnlyInterface, double controlDT, FramePoint initialPositionIfNotUsingVicon,
           SensorProcessor bodyReferenceFrameUpdater, List<BodyPositionEstimator> bodyPositionEstimators, List<BodyVelocityEstimator> bodyVelocityEstimators,
           BodyPositionEstimator bodyPositionEstimatorVicon, YoVariableRegistry estimatorRegistry, int bodyIMUIndex, boolean configureEveryTick)
   {
      initialPositionIfNotUsingVicon.checkReferenceFrameMatch(world);
      this.processedIMUSensors = processedIMUSensors;
      this.processedBodyPositionSensors = processedBodyPositionSensors;
      this.processedTimeSensorsReadOnlyInterface = processedTimeSensorsReadOnlyInterface;
      this.initialPositionIfNotUsingVicon = new FramePoint(initialPositionIfNotUsingVicon);
      this.bodyReferenceFrameUpdater = bodyReferenceFrameUpdater;
      this.bodyIMUIndex = bodyIMUIndex;
      this.configureEveryTick = configureEveryTick;

      int index = 0;
      for (BodyPositionEstimator bodyPositionEstimator : bodyPositionEstimators)
      {
         bodyPositionEstimatorIndices.put(bodyPositionEstimator, index++);
      }

      for (BodyVelocityEstimator bodyVelocityEstimator : bodyVelocityEstimators)
      {
         bodyVelocityEstimatorIndices.put(bodyVelocityEstimator, index++);
      }

      this.bodyPositionEstimatorVicon = bodyPositionEstimatorVicon;

      nMeasurements = bodyPositionEstimatorIndices.size() + bodyVelocityEstimatorIndices.size();
      configureKalmanFilters(controlDT);

      populateMatrices();

      setParameters();
      
      registry.addChild(estimatorRegistry);
   }

   public void initialize()
   {
      FrameVector covariance = new FrameVector(world, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      if (bodyPositionEstimatorVicon != null)
      {
         bodyPositionEstimatorVicon.estimateBodyPosition();

         FramePoint bodyPosition = new FramePoint(world);
         bodyPositionEstimatorVicon.getBodyPosition(bodyPosition);

         bodyPositionEstimatorVicon.getCovariance(covariance.getVector());
      }

      for (Direction direction : Direction.values())
      {
         double viconCovariance = covariance.get(direction);
         DenseMatrix64F P = new DenseMatrix64F(nStates, nStates);
         DenseMatrix64F x = new DenseMatrix64F(nStates, 1);

         if (!Double.isInfinite(viconCovariance))
         {
            P.set(positionIndex, positionIndex, viconCovariance);
            x.set(positionIndex, bodyPosition.get(direction));
         }
         else
         {
            x.set(positionIndex, initialPositionIfNotUsingVicon.get(direction));
         }

         kalmanFilters.get(direction).setState(x, P);
      }

      setProcessedSensors();
      updateReferenceFrames();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void update()
   {
      if (configureEveryTick) // For non real-time systems necessary
      {
         updateAllKalmanFilterConfigurations(processedTimeSensorsReadOnlyInterface.getDT());
      }
      
      updateInputs();
      updateMeasurements();
      setProcessNoiseCovariance();
      determineMeasurementNoiseCovariance();
      doKalmanFiltering();
      setProcessedSensors();
      updateReferenceFrames();
   }

   private final FramePoint tempEstimatedPosition = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FrameVector tempEstimatedVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   private void updateInputs()
   {
      // TODO: IMU is not always at body root joint, make this more general for arbitrary IMU position
      FrameVector bodyAcceleration = processedIMUSensors.getAcceleration(bodyIMUIndex);
      bodyAcceleration.checkReferenceFrameMatch(world);

      for (Direction direction : Direction.values())
      {
         inputs.get(direction).set(0, bodyAcceleration.get(direction));
      }
   }

   private void updateMeasurements()
   {
      for (BodyPositionEstimator bodyPositionEstimator : bodyPositionEstimatorIndices.keySet())
      {
         bodyPositionEstimator.estimateBodyPosition();
         bodyPositionEstimator.getBodyPosition(tempEstimatedPosition);
         tempEstimatedPosition.checkReferenceFrameMatch(world);
         int index = bodyPositionEstimatorIndices.get(bodyPositionEstimator);
         for (Direction direction : Direction.values())
         {
            double estimatedPosition = tempEstimatedPosition.get(direction);
            if (Double.isNaN(estimatedPosition))
               estimatedPosition = 0.0;    // otherwise the Kalman filter math won't work; happens e.g. when vicon is not turned on
            measurements.get(direction).set(index, estimatedPosition);
         }
      }

      for (BodyVelocityEstimator bodyVelocityEstimator : bodyVelocityEstimatorIndices.keySet())
      {
         bodyVelocityEstimator.estimateBodyVelocity();
         bodyVelocityEstimator.getBodyVelocity(tempEstimatedVelocity);
         tempEstimatedVelocity.checkReferenceFrameMatch(world);
         int index = bodyVelocityEstimatorIndices.get(bodyVelocityEstimator);
         for (Direction direction : Direction.values())
         {
            double estimatedVelocity = tempEstimatedVelocity.get(direction);
            if (Double.isNaN(estimatedVelocity))
               estimatedVelocity = 0.0;    // otherwise the Kalman filter math won't work; happens e.g. when vicon is not turned on
            measurements.get(direction).set(index, estimatedVelocity);
         }
      }
   }

   private void setProcessNoiseCovariance()
   {
      for (Direction direction : Direction.values())
      {
         DenseMatrix64F processCovariance = processCovariances.get(direction);
         processCovariance.set(velocityIndex, velocityIndex, accelerationCovariance.getDoubleValue());
         processCovariance.set(positionIndex, positionIndex, velocityCovariance.getDoubleValue());
         kalmanFilters.get(direction).setProcessNoiseCovariance(processCovariance);
      }
   }

   private final Tuple3d covariance = new Vector3d();

   private void determineMeasurementNoiseCovariance()
   {
      for (BodyPositionEstimator bodyPositionEstimator : bodyPositionEstimatorIndices.keySet())
      {
         bodyPositionEstimator.getCovariance(covariance);
         int index = bodyPositionEstimatorIndices.get(bodyPositionEstimator);
         measurementCovariances.get(Direction.X).set(index, index, covariance.getX());
         measurementCovariances.get(Direction.Y).set(index, index, covariance.getY());
         measurementCovariances.get(Direction.Z).set(index, index, covariance.getZ());
      }

      for (BodyVelocityEstimator bodyVelocityEstimator : bodyVelocityEstimatorIndices.keySet())
      {
         bodyVelocityEstimator.getCovariance(covariance);
         int index = bodyVelocityEstimatorIndices.get(bodyVelocityEstimator);
         measurementCovariances.get(Direction.X).set(index, index, covariance.getX());
         measurementCovariances.get(Direction.Y).set(index, index, covariance.getY());
         measurementCovariances.get(Direction.Z).set(index, index, covariance.getZ());
      }

      for (Direction direction : Direction.values())
      {
         kalmanFilters.get(direction).setMeasurementNoiseCovariance(measurementCovariances.get(direction));
      }
   }

   private void doKalmanFiltering()
   {
      for (Direction direction : Direction.values())
      {
         KalmanFilter kalmanFilter = kalmanFilters.get(direction);
         kalmanFilter.predict(inputs.get(direction));
         kalmanFilter.update(measurements.get(direction));
      }
   }

   private void setProcessedSensors()
   {
      DenseMatrix64F xState = kalmanFilters.get(Direction.X).getState();
      DenseMatrix64F yState = kalmanFilters.get(Direction.Y).getState();
      DenseMatrix64F zState = kalmanFilters.get(Direction.Z).getState();

      bodyPosition.set(xState.get(positionIndex), yState.get(positionIndex), zState.get(positionIndex));
      bodyVelocity.set(xState.get(velocityIndex), yState.get(velocityIndex), zState.get(velocityIndex));

      processedBodyPositionSensors.setBodyPosition(bodyPosition);
      processedBodyPositionSensors.setBodyVelocity(bodyVelocity);
   }

   private void updateReferenceFrames()
   {
      bodyReferenceFrameUpdater.update();
   }

   private void configureKalmanFilters(double controlDT)
   {
      for (Direction direction : Direction.values())
      {
         YoKalmanFilter kalmanFilter = new YoKalmanFilter("kalman" + direction, registry);
         updateKalmanFilterConfiguration(kalmanFilter, controlDT);
         kalmanFilters.put(direction, kalmanFilter);
      }
   }
   
   private void updateKalmanFilterConfiguration(KalmanFilter kalmanFilterToConfigure, double controlDT)
   {
      DenseMatrix64F stateTransitionMatrix = new DenseMatrix64F(nStates, nStates);
      stateTransitionMatrix.set(positionIndex, positionIndex, 1.0);
      stateTransitionMatrix.set(positionIndex, velocityIndex, controlDT);
      stateTransitionMatrix.set(velocityIndex, positionIndex, 0.0);
      stateTransitionMatrix.set(velocityIndex, velocityIndex, 1.0);

      DenseMatrix64F inputMatrix = new DenseMatrix64F(nStates, nInputs);
      inputMatrix.set(positionIndex, 0, 0.0);
      inputMatrix.set(velocityIndex, 0, controlDT);

      DenseMatrix64F outputMatrix = new DenseMatrix64F(nMeasurements, nStates);
      for (BodyPositionEstimator bodyPositionEstimator : bodyPositionEstimatorIndices.keySet())
      {
         int i = bodyPositionEstimatorIndices.get(bodyPositionEstimator);
         outputMatrix.set(i, positionIndex, 1.0);
      }

      for (BodyVelocityEstimator bodyVelocityEstimator : bodyVelocityEstimatorIndices.keySet())
      {
         int i = bodyVelocityEstimatorIndices.get(bodyVelocityEstimator);
         outputMatrix.set(i, velocityIndex, 1.0);
      }
      
      kalmanFilterToConfigure.configure(stateTransitionMatrix, inputMatrix, outputMatrix);
   }
   
   private void updateAllKalmanFilterConfigurations(double controlDT)
   {
      for (Direction direction : Direction.values())
      {
         updateKalmanFilterConfiguration(kalmanFilters.get(direction), controlDT);
      }
   }

   private void populateMatrices()
   {
      for (Direction direction : Direction.values())
      {
         processCovariances.put(direction, new DenseMatrix64F(nStates, nStates));
         measurementCovariances.put(direction, new DenseMatrix64F(nMeasurements, nMeasurements));
         inputs.put(direction, new DenseMatrix64F(nInputs, 1));
         measurements.put(direction, new DenseMatrix64F(nMeasurements, 1));
      }
   }

   private void setParameters()
   {
      accelerationCovariance.set(0.0011);    // obtained from a data set where the robot was in a fixed position. average of x, y, and z
      velocityCovariance.set(1e-4);    // sort of a fudge factor
   }
}
