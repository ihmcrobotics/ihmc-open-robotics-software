package us.ihmc.sensorProcessing.stateEstimation;

import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.kalman.KalmanFilter;
import us.ihmc.kalman.YoKalmanFilter;
import us.ihmc.robotics.robotController.SensorProcessor;
import us.ihmc.sensorProcessing.sensors.ProcessedBodyPositionSensorsWriteOnlyInterface;
import us.ihmc.sensorProcessing.sensors.ProcessedIMUSensorsReadOnlyInterface;
import us.ihmc.sensorProcessing.sensors.ProcessedTimeSensorsReadOnlyInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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

   private final EnumMap<Axis, KalmanFilter> kalmanFilters = new EnumMap<Axis, KalmanFilter>(Axis.class);

   private final EnumMap<Axis, DenseMatrix64F> processCovariances = new EnumMap<Axis, DenseMatrix64F>(Axis.class);
   private final EnumMap<Axis, DenseMatrix64F> measurementCovariances = new EnumMap<Axis, DenseMatrix64F>(Axis.class);
   private final EnumMap<Axis, DenseMatrix64F> inputs = new EnumMap<Axis, DenseMatrix64F>(Axis.class);
   private final EnumMap<Axis, DenseMatrix64F> measurements = new EnumMap<Axis, DenseMatrix64F>(Axis.class);

   private final YoDouble accelerationCovariance = new YoDouble("accelerationCovariance", registry);
   private final YoDouble velocityCovariance = new YoDouble("velocityCovariance", registry);

   private final FramePoint3D bodyPosition = new FramePoint3D(world);
   private final FrameVector3D bodyVelocity = new FrameVector3D(world);
   private final FramePoint3D initialPositionIfNotUsingVicon;

   private final int positionIndex = 0;
   private final int velocityIndex = 1;
   private final int nStates = 2;
   private final int nInputs = 1;
   private int nMeasurements;
   
   private final int bodyIMUIndex;
   private final boolean configureEveryTick;

   public BodyPositionAndVelocityEstimatorKalman(ProcessedIMUSensorsReadOnlyInterface processedIMUSensors,
           ProcessedBodyPositionSensorsWriteOnlyInterface processedBodyPositionSensors, ProcessedTimeSensorsReadOnlyInterface processedTimeSensorsReadOnlyInterface, double controlDT, FramePoint3D initialPositionIfNotUsingVicon,
           SensorProcessor bodyReferenceFrameUpdater, List<BodyPositionEstimator> bodyPositionEstimators, List<BodyVelocityEstimator> bodyVelocityEstimators,
           BodyPositionEstimator bodyPositionEstimatorVicon, YoVariableRegistry estimatorRegistry, int bodyIMUIndex, boolean configureEveryTick)
   {
      initialPositionIfNotUsingVicon.checkReferenceFrameMatch(world);
      this.processedIMUSensors = processedIMUSensors;
      this.processedBodyPositionSensors = processedBodyPositionSensors;
      this.processedTimeSensorsReadOnlyInterface = processedTimeSensorsReadOnlyInterface;
      this.initialPositionIfNotUsingVicon = new FramePoint3D(initialPositionIfNotUsingVicon);
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
      FrameVector3D covariance = new FrameVector3D(world, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      if (bodyPositionEstimatorVicon != null)
      {
         bodyPositionEstimatorVicon.estimateBodyPosition();

         FramePoint3D bodyPosition = new FramePoint3D(world);
         bodyPositionEstimatorVicon.getBodyPosition(bodyPosition);

         bodyPositionEstimatorVicon.getCovariance(covariance.getVector());
      }

      for (Axis axis : Axis.values())
      {
         double viconCovariance = covariance.getElement(axis.ordinal());
         DenseMatrix64F P = new DenseMatrix64F(nStates, nStates);
         DenseMatrix64F x = new DenseMatrix64F(nStates, 1);

         if (!Double.isInfinite(viconCovariance))
         {
            P.set(positionIndex, positionIndex, viconCovariance);
            x.set(positionIndex, bodyPosition.getElement(axis.ordinal()));
         }
         else
         {
            x.set(positionIndex, initialPositionIfNotUsingVicon.getElement(axis.ordinal()));
         }

         kalmanFilters.get(axis).setState(x, P);
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

   private final FramePoint3D tempEstimatedPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D tempEstimatedVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private void updateInputs()
   {
      // TODO: IMU is not always at body root joint, make this more general for arbitrary IMU position
      FrameVector3D bodyAcceleration = processedIMUSensors.getAcceleration(bodyIMUIndex);
      bodyAcceleration.checkReferenceFrameMatch(world);

      for (Axis axis : Axis.values())
      {
         inputs.get(axis).set(0, bodyAcceleration.getElement(axis.ordinal()));
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
         for (Axis axis : Axis.values())
         {
            double estimatedPosition = tempEstimatedPosition.getElement(axis.ordinal());
            if (Double.isNaN(estimatedPosition))
               estimatedPosition = 0.0;    // otherwise the Kalman filter math won't work; happens e.g. when vicon is not turned on
            measurements.get(axis).set(index, estimatedPosition);
         }
      }

      for (BodyVelocityEstimator bodyVelocityEstimator : bodyVelocityEstimatorIndices.keySet())
      {
         bodyVelocityEstimator.estimateBodyVelocity();
         bodyVelocityEstimator.getBodyVelocity(tempEstimatedVelocity);
         tempEstimatedVelocity.checkReferenceFrameMatch(world);
         int index = bodyVelocityEstimatorIndices.get(bodyVelocityEstimator);
         for (Axis axis : Axis.values())
         {
            double estimatedVelocity = tempEstimatedVelocity.getElement(axis.ordinal());
            if (Double.isNaN(estimatedVelocity))
               estimatedVelocity = 0.0;    // otherwise the Kalman filter math won't work; happens e.g. when vicon is not turned on
            measurements.get(axis).set(index, estimatedVelocity);
         }
      }
   }

   private void setProcessNoiseCovariance()
   {
      for (Axis axis : Axis.values())
      {
         DenseMatrix64F processCovariance = processCovariances.get(axis);
         processCovariance.set(velocityIndex, velocityIndex, accelerationCovariance.getDoubleValue());
         processCovariance.set(positionIndex, positionIndex, velocityCovariance.getDoubleValue());
         kalmanFilters.get(axis).setProcessNoiseCovariance(processCovariance);
      }
   }

   private final Tuple3DBasics covariance = new Vector3D();

   private void determineMeasurementNoiseCovariance()
   {
      for (BodyPositionEstimator bodyPositionEstimator : bodyPositionEstimatorIndices.keySet())
      {
         bodyPositionEstimator.getCovariance(covariance);
         int index = bodyPositionEstimatorIndices.get(bodyPositionEstimator);
         measurementCovariances.get(Axis.X).set(index, index, covariance.getX());
         measurementCovariances.get(Axis.Y).set(index, index, covariance.getY());
         measurementCovariances.get(Axis.Z).set(index, index, covariance.getZ());
      }

      for (BodyVelocityEstimator bodyVelocityEstimator : bodyVelocityEstimatorIndices.keySet())
      {
         bodyVelocityEstimator.getCovariance(covariance);
         int index = bodyVelocityEstimatorIndices.get(bodyVelocityEstimator);
         measurementCovariances.get(Axis.X).set(index, index, covariance.getX());
         measurementCovariances.get(Axis.Y).set(index, index, covariance.getY());
         measurementCovariances.get(Axis.Z).set(index, index, covariance.getZ());
      }

      for (Axis axis : Axis.values())
      {
         kalmanFilters.get(axis).setMeasurementNoiseCovariance(measurementCovariances.get(axis));
      }
   }

   private void doKalmanFiltering()
   {
      for (Axis axis : Axis.values())
      {
         KalmanFilter kalmanFilter = kalmanFilters.get(axis);
         kalmanFilter.predict(inputs.get(axis));
         kalmanFilter.update(measurements.get(axis));
      }
   }

   private void setProcessedSensors()
   {
      DenseMatrix64F xState = kalmanFilters.get(Axis.X).getState();
      DenseMatrix64F yState = kalmanFilters.get(Axis.Y).getState();
      DenseMatrix64F zState = kalmanFilters.get(Axis.Z).getState();

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
      for (Axis axis : Axis.values())
      {
         YoKalmanFilter kalmanFilter = new YoKalmanFilter("kalman" + axis, registry);
         updateKalmanFilterConfiguration(kalmanFilter, controlDT);
         kalmanFilters.put(axis, kalmanFilter);
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
      for (Axis axis : Axis.values())
      {
         updateKalmanFilterConfiguration(kalmanFilters.get(axis), controlDT);
      }
   }

   private void populateMatrices()
   {
      for (Axis axis : Axis.values())
      {
         processCovariances.put(axis, new DenseMatrix64F(nStates, nStates));
         measurementCovariances.put(axis, new DenseMatrix64F(nMeasurements, nMeasurements));
         inputs.put(axis, new DenseMatrix64F(nInputs, 1));
         measurements.put(axis, new DenseMatrix64F(nMeasurements, 1));
      }
   }

   private void setParameters()
   {
      accelerationCovariance.set(0.0011);    // obtained from a data set where the robot was in a fixed position. average of x, y, and z
      velocityCovariance.set(1e-4);    // sort of a fudge factor
   }
}
