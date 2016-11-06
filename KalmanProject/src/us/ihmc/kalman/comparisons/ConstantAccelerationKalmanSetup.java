package us.ihmc.kalman.comparisons;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.kalman.YoKalmanFilter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;

public class ConstantAccelerationKalmanSetup implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ExampleFunctionController sensors;

   private YoKalmanFilter yoKalmanFilter;

   private final int positionIndex = 0;
   private final int velocityIndex = 1;
   private final int accelerationIndex = 2;
   
   private final int positionMeasurementIndex = 0;
   private final int accelerationMeasurementIndex = 1;
   
   private int nStates = 3;
   private int nInputs = 1;//actually zero, just for the sake of null pointer exceptions
   private int nMeasurements = 2;

   private DenseMatrix64F processCovariance;
   private DenseMatrix64F inputs;
   private DenseMatrix64F measurements;
   private DenseMatrix64F measurementCovariance;
   
   private double ultrasonicsVariance = 1e-3;
   private double imuVariance = 0.01;

   private final boolean isNonRealTimeWithVariableDT;
   
   private double dt;

   public ConstantAccelerationKalmanSetup(ExampleFunctionController sensors, double controlDT, boolean isNonRealTimeWithVariableDT)
   {
      this.isNonRealTimeWithVariableDT = isNonRealTimeWithVariableDT;
      this.sensors = sensors;
      
      dt = controlDT;

      populateAndConfigureYoKalmanFilter(controlDT);
      populateMatrices();

   }

   private void populateAndConfigureYoKalmanFilter(double controlDT)
   {
      yoKalmanFilter = new YoKalmanFilter(name + "YoKalman", registry);
      updateYoKalmanFilterConfiguration(controlDT);
   }

   private void updateYoKalmanFilterConfiguration(double controlDT)
   {
      DenseMatrix64F modelStateEvolutionF = new DenseMatrix64F(nStates, nStates);
      modelStateEvolutionF.set(positionIndex, positionIndex, 1.0);
      modelStateEvolutionF.set(positionIndex, velocityIndex, controlDT);
      modelStateEvolutionF.set(positionIndex, accelerationIndex, controlDT * controlDT / 2.0);
      modelStateEvolutionF.set(velocityIndex, velocityIndex, 1.0);
      modelStateEvolutionF.set(velocityIndex, accelerationIndex, controlDT);
      modelStateEvolutionF.set(accelerationIndex, accelerationIndex, 1.0);

      DenseMatrix64F modelInputInfluenceG = new DenseMatrix64F(nStates, nInputs);

      DenseMatrix64F modelOutputH = new DenseMatrix64F(nMeasurements, nStates);
      modelOutputH.set(positionMeasurementIndex, positionIndex, 1.0);
      modelOutputH.set(accelerationMeasurementIndex, accelerationIndex, 1.0);

      yoKalmanFilter.configure(modelStateEvolutionF, modelInputInfluenceG, modelOutputH);
   }

   private void populateMatrices()
   {
      processCovariance = new DenseMatrix64F(nStates, nStates);
      measurementCovariance = new DenseMatrix64F(nMeasurements, nMeasurements);
      inputs = new DenseMatrix64F(nInputs, 1);
      measurements = new DenseMatrix64F(nMeasurements, 1);
   }

   public void initialize()
   {
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
      return name;
   }

   public void doControl()
   {
      updateYoKalmanFilter();
      updateInputs();
      updateMeasurements();
      setProcessNoiseCovariance();
      setMeasurementNoiseCovariance();

      doKalmanFiltering();
   }

   private void updateYoKalmanFilter()
   {
      if (isNonRealTimeWithVariableDT)
         updateYoKalmanFilterConfiguration(dt);
   }

   private void updateInputs()
   {
   }

   private void updateMeasurements()
   {
      measurements.set(positionMeasurementIndex, 0, sensors.getValue());
      measurements.set(accelerationMeasurementIndex, 0, sensors.getSecondDerivative());
   }

   private void setProcessNoiseCovariance()
   {      
      // 1.5g change received in a single tick used as Variance
      //double estimatedAccelerationDifferenceInOneTick = 1.5 * 9.81 / dt;
      double maxModeledJerk = 4.0e-3; //theoretical result 4.0e-6
      
      processCovariance.set(positionIndex, positionIndex, Math.pow(maxModeledJerk * dt * dt * dt / 6.0, 2.0));
      processCovariance.set(velocityIndex, velocityIndex, Math.pow(maxModeledJerk * dt * dt / 2.0, 2.0));
      processCovariance.set(accelerationIndex, accelerationIndex, Math.pow(maxModeledJerk * dt , 2.0));
      yoKalmanFilter.setProcessNoiseCovariance(processCovariance);
   }

   private void setMeasurementNoiseCovariance()
   {
      if (sensors.isPositionMeasurementUpdated())
         measurementCovariance.set(positionMeasurementIndex, 0, ultrasonicsVariance);
      else
         measurementCovariance.set(positionMeasurementIndex, 0, Double.POSITIVE_INFINITY);
      
      measurementCovariance.set(accelerationMeasurementIndex, 1, imuVariance);
      yoKalmanFilter.setMeasurementNoiseCovariance(measurementCovariance);
   }

   private void doKalmanFiltering()
   {
      yoKalmanFilter.predict(inputs);
      yoKalmanFilter.update(measurements);
   }
}
