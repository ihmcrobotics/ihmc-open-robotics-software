package us.ihmc.kalman.comparisons;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.kalman.YoKalmanFilter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;

public class ImuAsInputKalmanSetup implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ExampleFunctionController sensors;

   private YoKalmanFilter yoKalmanFilter;

   private final int positionIndex = 0;
   private final int velocityIndex = 1;
   private int nStates = 2;
   private int nInputs = 1;
   private int nMeasurements = 1;

   private DenseMatrix64F processCovariance;
   private DenseMatrix64F inputs;
   private DenseMatrix64F measurements;
   private DenseMatrix64F measurementCovariance;

   private double velocityModelCovariance = 0.0011;
   private double positionModelCovariance = 1e-4;
   private double defaultCovariance = 1e-3;

   private final boolean isNonRealTimeWithVariableDT;
   private double dt;
   
   public ImuAsInputKalmanSetup(ExampleFunctionController sensors, double controlDT, boolean isNonRealTimeWithVariableDT)
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
      modelStateEvolutionF.set(velocityIndex, velocityIndex, 1.0);

      DenseMatrix64F modelInputInfluenceG = new DenseMatrix64F(nStates, nInputs);
      modelInputInfluenceG.set(velocityIndex, positionIndex, controlDT * controlDT / 2.0);
      modelInputInfluenceG.set(velocityIndex, positionIndex, controlDT);

      DenseMatrix64F modelOutputH = new DenseMatrix64F(nMeasurements, nStates);
      modelOutputH.set(positionIndex, positionIndex, 1.0);

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
      inputs.set(0, 0, sensors.getSecondDerivative());
   }

   private void updateMeasurements()
   {
      measurements.set(0, 0, sensors.getValue());
   }

   private void setProcessNoiseCovariance()
   {
      
      double maxModeledJerk = 4.0e-3;
      processCovariance.set(positionIndex, positionIndex, Math.pow(maxModeledJerk * dt * dt * dt / 6.0, 2.0));
      processCovariance.set(velocityIndex, velocityIndex, Math.pow(maxModeledJerk * dt * dt / 2.0, 2.0));
//      processCovariance.set(positionIndex, positionIndex, positionModelCovariance);
//      processCovariance.set(velocityIndex, velocityIndex, velocityModelCovariance);
      yoKalmanFilter.setProcessNoiseCovariance(processCovariance);
   }

   private void setMeasurementNoiseCovariance()
   {
      if (sensors.isPositionMeasurementUpdated())
         measurementCovariance.set(0, 0, defaultCovariance);
      else
         measurementCovariance.set(0, 0, Double.POSITIVE_INFINITY);

      yoKalmanFilter.setMeasurementNoiseCovariance(measurementCovariance);
   }

   private void doKalmanFiltering()
   {
      yoKalmanFilter.predict(inputs);
      yoKalmanFilter.update(measurements);
   }
}
