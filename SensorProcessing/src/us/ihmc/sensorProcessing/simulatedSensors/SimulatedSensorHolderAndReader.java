package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.utilities.ForceSensorDefinition;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.maps.ObjectObjectMap;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.filter.BacklashCompensatingVelocityYoVariable;

public class SimulatedSensorHolderAndReader implements SensorReader, Runnable
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   private final IntegerYoVariable step = new IntegerYoVariable("step", registry);
   
   private final ReentrantLock lock = new ReentrantLock();
   private final Condition simulationDoneCondition = lock.newCondition();
   private final Condition simulationContinueCondition = lock.newCondition();
   private boolean simulationIsDone = false;
   private boolean simulationContinue = false;
   
   private boolean started = false;

   private final double estimatorDT;
   private final long estimateDTinNs;

   private volatile ControllerDispatcher controllerDispatcher;

   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointPositionSensor> jointPositionSensors = new ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointPositionSensor>();
   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointVelocitySensor> jointVelocitySensors = new ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointVelocitySensor>();

   private final ObjectObjectMap<IMUDefinition, SimulatedOrientationSensorFromRobot> orientationSensors = new ObjectObjectMap<IMUDefinition, SimulatedOrientationSensorFromRobot>();

   private final ObjectObjectMap<IMUDefinition, SimulatedAngularVelocitySensorFromRobot> angularVelocitySensors = new ObjectObjectMap<IMUDefinition, SimulatedAngularVelocitySensorFromRobot>();
   private final ObjectObjectMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot> linearAccelerationSensors = new ObjectObjectMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot>();

   private final ObjectObjectMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new ObjectObjectMap<ForceSensorDefinition, WrenchCalculatorInterface>();

   private final SensorProcessing sensorProcessing;

   private ForceSensorDataHolder forceSensorDataHolder;

   /*
    * Multithreading support
    */
   private final DispatcherExecutor dispatcherExecutor = new DispatcherExecutor();

   private final BooleanYoVariable useFiniteDifferencesForVelocities;
   private final DoubleYoVariable alphaFiniteDifferences;
   private final DoubleYoVariable slopTimeFiniteDifferences;

   private ObjectObjectMap<OneDoFJoint, BacklashCompensatingVelocityYoVariable> finiteDifferenceVelocities;

   public SimulatedSensorHolderAndReader(SensorFilterParameters sensorFilterParameters, StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions, SensorNoiseParameters sensorNoiseParameters, YoVariableRegistry simulationRegistry)
   {
      sensorProcessing = new SensorProcessing(stateEstimatorSensorDefinitions, sensorFilterParameters, sensorNoiseParameters, registry);
      
      this.estimatorDT = sensorFilterParameters.getEstimatorDT();
      this.estimateDTinNs = TimeTools.secondsToNanoSeconds(estimatorDT);
      step.set(29831);

      useFiniteDifferencesForVelocities = new BooleanYoVariable("useFiniteDifferencesForVelocities", registry);
      alphaFiniteDifferences = new DoubleYoVariable("alphaFiniteDifferences", registry);
      slopTimeFiniteDifferences = new DoubleYoVariable("slopTimeFiniteDifferences", registry);

      alphaFiniteDifferences.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(
            sensorFilterParameters.getJointVelocityFilterFrequencyInHertz(), estimatorDT));
      slopTimeFiniteDifferences.set(sensorFilterParameters.getJointVelocitySlopTimeForBacklashCompensation());
      useFiniteDifferencesForVelocities.set(false);

      if (simulationRegistry != null)
      {
         simulationRegistry.addChild(registry);
      }
   }

   public void addJointPositionSensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointPositionSensor jointPositionSensor)
   {
      jointPositionSensors.add(oneDoFJoint, jointPositionSensor);
   }

   public void addJointVelocitySensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointVelocitySensor jointVelocitySensor)
   {
      jointVelocitySensors.add(oneDoFJoint, jointVelocitySensor);

      if (finiteDifferenceVelocities == null)
         finiteDifferenceVelocities = new ObjectObjectMap<OneDoFJoint, BacklashCompensatingVelocityYoVariable>();

      BacklashCompensatingVelocityYoVariable finiteDifferenceVelocity = new BacklashCompensatingVelocityYoVariable("fd_qd_" + oneDoFJoint.getName(), "",
            alphaFiniteDifferences, estimatorDT, slopTimeFiniteDifferences, registry);
      finiteDifferenceVelocities.add(oneDoFJoint, finiteDifferenceVelocity);
   }

   public void addOrientationSensorPort(IMUDefinition imuDefinition, SimulatedOrientationSensorFromRobot orientationSensor)
   {
      orientationSensors.add(imuDefinition, orientationSensor);
   }

   public void addAngularVelocitySensorPort(IMUDefinition imuDefinition, SimulatedAngularVelocitySensorFromRobot angularVelocitySensor)
   {
      angularVelocitySensors.add(imuDefinition, angularVelocitySensor);
   }

   public void addLinearAccelerationSensorPort(IMUDefinition imuDefinition, SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor)
   {
      linearAccelerationSensors.add(imuDefinition, linearAccelerationSensor);
   }

   public void addForceTorqueSensorPort(ForceSensorDefinition forceSensorDefinition, WrenchCalculatorInterface groundContactPointBasedWrenchCalculator)
   {
      forceTorqueSensors.add(forceSensorDefinition, groundContactPointBasedWrenchCalculator);
   }

   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sensorProcessing;
   }

   public void run()
   {
      if(!started)
      {
         dispatcherExecutor.start();
         started = true;
      }
      
      if (controllerDispatcher != null)
      {
         controllerDispatcher.waitUntilComputationIsDone();
      }
      lock.lock();
      simulationIsDone = true;
      simulationDoneCondition.signalAll();

      while(!simulationContinue)
      {
         try
         {
            simulationContinueCondition.await();            
         }
         catch (InterruptedException e)
         {
         }
      }
      simulationContinue = false;
      lock.unlock();
   }

   public void setControllerDispatcher(ControllerDispatcher controllerDispatcher)
   {
      this.controllerDispatcher = controllerDispatcher;
   }

   public void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder)
   {
      this.forceSensorDataHolder = forceSensorDataHolder;
   }

   private class DispatcherExecutor extends Thread
   {

      public void run()
      {
         while (true)
         {
            
            // Wait for simulation to finish

            lock.lock();
            while(!simulationIsDone)
            {
               try
               {
                  simulationDoneCondition.await();
               }
               catch (InterruptedException e)
               {
               }
            }
            simulationIsDone = false;
            lock.unlock();
            
            for (int i = 0; i < jointPositionSensors.getLength(); i++)
            {
               SimulatedOneDoFJointPositionSensor simulatedOneDoFJointPositionSensor = jointPositionSensors.getSecond(i);
               simulatedOneDoFJointPositionSensor.startComputation();
               simulatedOneDoFJointPositionSensor.waitUntilComputationIsDone();
               Double value = simulatedOneDoFJointPositionSensor.getJointPositionOutputPort().getData();
               sensorProcessing.setJointPositionSensorValue(jointPositionSensors.getFirst(i), value);

               BacklashCompensatingVelocityYoVariable finiteDifferenceVelocity = finiteDifferenceVelocities.getSecond(i);
               finiteDifferenceVelocity.update(value);

               if (useFiniteDifferencesForVelocities.getBooleanValue())
               {
                  sensorProcessing.setJointVelocitySensorValue(jointVelocitySensors.getFirst(i), finiteDifferenceVelocity.getDoubleValue());
               }
            }

            if (!useFiniteDifferencesForVelocities.getBooleanValue())
            {
               for (int i = 0; i < jointVelocitySensors.getLength(); i++)
               {
                  SimulatedOneDoFJointVelocitySensor simulatedOneDoFJointVelocitySensor = jointVelocitySensors.getSecond(i);
                  simulatedOneDoFJointVelocitySensor.startComputation();
                  simulatedOneDoFJointVelocitySensor.waitUntilComputationIsDone();
                  Double value = simulatedOneDoFJointVelocitySensor.getJointVelocityOutputPort().getData();
                  sensorProcessing.setJointVelocitySensorValue(jointVelocitySensors.getFirst(i), value);
               }
            }

            for (int i = 0; i < orientationSensors.getLength(); i++)
            {
               SimulatedOrientationSensorFromRobot orientationSensor = orientationSensors.getSecond(i);
               orientationSensor.startComputation();
               orientationSensor.waitUntilComputationIsDone();
               Matrix3d value = orientationSensor.getOrientationOutputPort().getData();
               sensorProcessing.setOrientationSensorValue(orientationSensors.getFirst(i), value);
            }

            for (int i = 0; i < angularVelocitySensors.getLength(); i++)
            {
               SimulatedAngularVelocitySensorFromRobot angularVelocitySensor = angularVelocitySensors.getSecond(i);
               angularVelocitySensor.startComputation();
               angularVelocitySensor.waitUntilComputationIsDone();
               Vector3d value = angularVelocitySensor.getAngularVelocityOutputPort().getData();
               sensorProcessing.setAngularVelocitySensorValue(angularVelocitySensors.getFirst(i), value);
            }

            for (int i = 0; i < linearAccelerationSensors.getLength(); i++)
            {

               SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor = linearAccelerationSensors.getSecond(i);
               linearAccelerationSensor.startComputation();
               linearAccelerationSensor.waitUntilComputationIsDone();
               Vector3d value = linearAccelerationSensor.getLinearAccelerationOutputPort().getData();
               sensorProcessing.setLinearAccelerationSensorValue(linearAccelerationSensors.getFirst(i), value);
            }

            if (forceSensorDataHolder != null)
            {
               for (int i = 0; i < forceTorqueSensors.getLength(); i++)
               {
                  final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensors.getSecond(i);
                  forceTorqueSensor.calculate();
                  forceSensorDataHolder.setForceSensorValue(forceTorqueSensors.getFirst(i), forceTorqueSensor.getWrench());
               }
            }

            sensorProcessing.startComputation();
            sensorProcessing.waitUntilComputationIsDone();

            step.increment();

            // Signal sim to continue

            lock.lock();
            simulationContinue = true;
            simulationContinueCondition.signalAll();
            lock.unlock();

            if(controllerDispatcher != null)
            {
               controllerDispatcher.startEstimator(estimateDTinNs * step.getIntegerValue(), System.nanoTime());               
            }
         }
      }
   }
}
