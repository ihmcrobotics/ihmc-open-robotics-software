package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.stateEstimation.JointAndIMUSensorDataSource;
import us.ihmc.utilities.ForceSensorDefinition;
import us.ihmc.utilities.IMUDefinition;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.maps.ObjectObjectMap;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

import com.yobotics.simulationconstructionset.IntegerYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SimulatedSensorHolderAndReader implements SensorReader, Runnable
{
   private final static boolean RUN_MULTITHREADED = true;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("DRCPerfectSensorReader");
   private final IntegerYoVariable step = new IntegerYoVariable("step", registry);
   private final long estimateDTinNs;
   
   private ControllerDispatcher controllerDispatcher;
   
   
   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointPositionSensor> jointPositionSensors = new ObjectObjectMap<OneDoFJoint,
                                                                                                          SimulatedOneDoFJointPositionSensor>();
   private final ObjectObjectMap<OneDoFJoint, SimulatedOneDoFJointVelocitySensor> jointVelocitySensors = new ObjectObjectMap<OneDoFJoint,
                                                                                                          SimulatedOneDoFJointVelocitySensor>();

   private final ObjectObjectMap<IMUDefinition, SimulatedOrientationSensorFromRobot> orientationSensors = new ObjectObjectMap<IMUDefinition,
                                                                                                           SimulatedOrientationSensorFromRobot>();

   private final ObjectObjectMap<IMUDefinition, SimulatedAngularVelocitySensorFromRobot> angularVelocitySensors = new ObjectObjectMap<IMUDefinition,
                                                                                                                   SimulatedAngularVelocitySensorFromRobot>();
   private final ObjectObjectMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot> linearAccelerationSensors =
      new ObjectObjectMap<IMUDefinition, SimulatedLinearAccelerationSensorFromRobot>();

   private final ObjectObjectMap<ForceSensorDefinition, WrenchCalculatorInterface> forceTorqueSensors = new ObjectObjectMap<ForceSensorDefinition, WrenchCalculatorInterface>();
   

   private JointAndIMUSensorDataSource jointAndIMUSensorDataSource;
   
   
   private ForceSensorDataHolder forceSensorDataHolder;
   
   /*
    * Multithreading support
    */
   private final DispatcherExecutor dispatcherExecutor = new DispatcherExecutor();
   private final ExecutorService dispatcherExecutorPool = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory("dispatcherExecutor"));
   private Future<?> dispatcherFuture;

   
   public SimulatedSensorHolderAndReader(double estimateDT, YoVariableRegistry parentRegistry)
   {
      this.estimateDTinNs = TimeTools.secondsToNanoSeconds(estimateDT);
      step.set(29831);
      
      if(parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public void addJointPositionSensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointPositionSensor jointPositionSensor)
   {
      jointPositionSensors.add(oneDoFJoint, jointPositionSensor);
   }

   public void addJointVelocitySensorPort(OneDoFJoint oneDoFJoint, SimulatedOneDoFJointVelocitySensor jointVelocitySensor)
   {
      jointVelocitySensors.add(oneDoFJoint, jointVelocitySensor);
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


   public void setJointAndIMUSensorDataSource(JointAndIMUSensorDataSource jointAndIMUSensorDataSource)
   {
      this.jointAndIMUSensorDataSource = jointAndIMUSensorDataSource;
   }

   public void run()
   {
      if(controllerDispatcher != null)
      {
         if(RUN_MULTITHREADED)
         {
            if(dispatcherFuture != null)
            {
               try
               {
                  dispatcherFuture.get();
               }
               catch (InterruptedException e)
               {
                  System.err.println(e);
               }
               catch (ExecutionException e)
               {
                  throw new RuntimeException(e);
               }
            }
         }
         controllerDispatcher.waitUntilComputationIsDone();
      }
      step.increment();
      
      
      for(int i = 0; i < jointPositionSensors.getLength(); i++)
      {
         SimulatedOneDoFJointPositionSensor simulatedOneDoFJointPositionSensor = jointPositionSensors.getSecond(i);
         simulatedOneDoFJointPositionSensor.startComputation();
         simulatedOneDoFJointPositionSensor.waitUntilComputationIsDone();
         Double value = simulatedOneDoFJointPositionSensor.getJointPositionOutputPort().getData();
         jointAndIMUSensorDataSource.setJointPositionSensorValue(jointPositionSensors.getFirst(i), value);
      }

      for(int i = 0; i < jointVelocitySensors.getLength(); i++)
      {
         SimulatedOneDoFJointVelocitySensor simulatedOneDoFJointVelocitySensor = jointVelocitySensors.getSecond(i);
         simulatedOneDoFJointVelocitySensor.startComputation();
         simulatedOneDoFJointVelocitySensor.waitUntilComputationIsDone();
         Double value = simulatedOneDoFJointVelocitySensor.getJointVelocityOutputPort().getData();
         jointAndIMUSensorDataSource.setJointVelocitySensorValue(jointVelocitySensors.getFirst(i), value);
      }

      for(int i = 0; i < orientationSensors.getLength(); i++)
      {
         SimulatedOrientationSensorFromRobot orientationSensor = orientationSensors.getSecond(i);
         orientationSensor.startComputation();
         orientationSensor.waitUntilComputationIsDone();
         Matrix3d value = orientationSensor.getOrientationOutputPort().getData();
         jointAndIMUSensorDataSource.setOrientationSensorValue(orientationSensors.getFirst(i), value);
      }

      for(int i = 0; i < angularVelocitySensors.getLength(); i++)
      {
         SimulatedAngularVelocitySensorFromRobot angularVelocitySensor = angularVelocitySensors.getSecond(i);
         angularVelocitySensor.startComputation();
         angularVelocitySensor.waitUntilComputationIsDone();
         Vector3d value = angularVelocitySensor.getAngularVelocityOutputPort().getData();
         jointAndIMUSensorDataSource.setAngularVelocitySensorValue(angularVelocitySensors.getFirst(i), value);
      }

      for(int i = 0; i < linearAccelerationSensors.getLength(); i++)
      {

         SimulatedLinearAccelerationSensorFromRobot linearAccelerationSensor = linearAccelerationSensors.getSecond(i);
         linearAccelerationSensor.startComputation();
         linearAccelerationSensor.waitUntilComputationIsDone();
         Vector3d value = linearAccelerationSensor.getLinearAccelerationOutputPort().getData();
         jointAndIMUSensorDataSource.setLinearAccelerationSensorValue(linearAccelerationSensors.getFirst(i), value);
      }
      
      
      if(forceSensorDataHolder != null)
      {
         for(int i = 0; i < forceTorqueSensors.getLength(); i++)
         {
            final WrenchCalculatorInterface forceTorqueSensor = forceTorqueSensors.getSecond(i);
            forceTorqueSensor.startComputation();
            forceTorqueSensor.waitUntilComputationIsDone();  
            forceSensorDataHolder.setForceSensorValue(forceTorqueSensors.getFirst(i), forceTorqueSensor.getForceSensorOutputPort().getData());
         }
      }
      
      
      
      if(controllerDispatcher != null)
      {
         if(RUN_MULTITHREADED)
         {
            dispatcherFuture = dispatcherExecutorPool.submit(dispatcherExecutor);
         }
         else
         {
            dispatcherExecutor.run();
         }
      }
   }

   public void setControllerDispatcher(ControllerDispatcher controllerDispatcher)
   {
      this.controllerDispatcher = controllerDispatcher;
   }

   
   public void setForceSensorDataHolder(ForceSensorDataHolder forceSensorDataHolder)
   {
      this.forceSensorDataHolder = forceSensorDataHolder;
   }
   
   private class DispatcherExecutor implements Runnable
   {

      public void run()
      {
         controllerDispatcher.startEstimator(estimateDTinNs * step.getIntegerValue());
      }
      
   }
}
