package us.ihmc.darpaRoboticsChallenge.networkProcessor.kinematicsToolboxModule;

import java.util.Arrays;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.HumanoidFloatingRootJointRobot;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.util.PeriodicThreadScheduler;

public class KinematicToolboxDiagnosticEnvironment
{
   private final String robotName = "Floating Robot";
   private final RobotDescription robotDescription = new RobotDescription(robotName);
   private final ReferenceFrames referenceFrames = null;
   private final String threadName = "Non Realtime Scheduler";

   public KinematicToolboxDiagnosticEnvironment(DRCRobotModel drcRobotModel)
   {
      FullHumanoidRobotModel humanoidFullRobotModel = drcRobotModel.createFullRobotModel();
      HumanoidJointNameMap sdfJointNameMap = drcRobotModel.getJointMap();
      HumanoidFloatingRootJointRobot humanoidFloatingRobotModel = new HumanoidFloatingRootJointRobot(robotDescription, sdfJointNameMap);
      SDFPerfectSimulatedSensorReader sdfPerfectReader = new SDFPerfectSimulatedSensorReader(humanoidFloatingRobotModel, humanoidFullRobotModel,
                                                                                             referenceFrames);

      ForceSensorDefinition[] forceSensorDefinitionArray = humanoidFullRobotModel.getForceSensorDefinitions();
      List<ForceSensorDefinition> forceSensorDefinitionList = Arrays.asList(forceSensorDefinitionArray);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(forceSensorDefinitionList);
      JointConfigurationGatherer jointConfigurationGatherer = new JointConfigurationGatherer(humanoidFullRobotModel, forceSensorDataHolder);

      HumanoidGlobalDataProducer dataProducer = null;
      SensorOutputMapReadOnly sensorOutputMapReadOnly = initializeSensorOutputMapReadOnly();
      SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly = initializeSensorRawOutputMapReadOnly();
      RobotMotionStatusHolder robotMotionStatusFromController = new RobotMotionStatusHolder();
      IMUSensorReadOnly sensorInformation = initializeIMUSensorReadOnly();
      //PeriodicThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler(threadName);
//      DRCPoseCommunicator poseCommunicator = new DRCPoseCommunicator(humanoidFullRobotModel, jointConfigurationGatherer, sdfPerfectReader /* null */,
//                                                                     dataProducer, sensorOutputMapReadOnly, sensorRawOutputMapReadOnly,
//                                                                     robotMotionStatusFromController, sensorInformation, scheduler,
//                                                                     new IHMCCommunicationKryoNetClassList());
   }

   private IMUSensorReadOnly initializeIMUSensorReadOnly()
   {
      return new IMUSensorReadOnly()
      {
         
         @Override
         public String getSensorName()
         {
            return null;
         }
         
         @Override
         public void getOrientationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
         {
         }
         
         @Override
         public void getOrientationMeasurement(Matrix3f orientationToPack)
         {
         }
         
         @Override
         public void getOrientationMeasurement(Matrix3d orientationToPack)
         {
         }
         
         @Override
         public RigidBody getMeasurementLink()
         {
            return null;
         }
         
         @Override
         public ReferenceFrame getMeasurementFrame()
         {
            return null;
         }
         
         @Override
         public void getLinearAccelerationNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
         {
         }
         
         @Override
         public void getLinearAccelerationMeasurement(Vector3f linearAccelerationToPack)
         {
         }
         
         @Override
         public void getLinearAccelerationMeasurement(Vector3d linearAccelerationToPack)
         {
         }
         
         @Override
         public void getLinearAccelerationBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack)
         {
         }
         
         @Override
         public void getAngularVelocityNoiseCovariance(DenseMatrix64F noiseCovarianceToPack)
         {
         }
         
         @Override
         public void getAngularVelocityMeasurement(Vector3f angularVelocityToPack)
         {
         }
         
         @Override
         public void getAngularVelocityMeasurement(Vector3d angularVelocityToPack)
         {
         }
         
         @Override
         public void getAngularVelocityBiasProcessNoiseCovariance(DenseMatrix64F biasProcessNoiseCovarianceToPack)
         {
         }
      };
   }

   private SensorRawOutputMapReadOnly initializeSensorRawOutputMapReadOnly()
   {
      return new SensorRawOutputMapReadOnly()
      {
         
         @Override
         public long getVisionSensorTimestamp()
         {
            return 0;
         }
         
         @Override
         public long getTimestamp()
         {
            return 0;
         }
         
         @Override
         public long getSensorHeadPPSTimestamp()
         {
            return 0;
         }
         
         @Override
         public double getJointVelocityRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }
         
         @Override
         public double getJointTauRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }
         
         @Override
         public double getJointPositionRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }
         
         @Override
         public double getJointAccelerationRawOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }
         
         @Override
         public List<? extends IMUSensorReadOnly> getIMURawOutputs()
         {
            return null;
         }
         
         @Override
         public ForceSensorDataHolderReadOnly getForceSensorRawOutputs()
         {
            return null;
         }
         
         @Override
         public AuxiliaryRobotData getAuxiliaryRobotData()
         {
            return null;
         }
      };
   }

   private SensorOutputMapReadOnly initializeSensorOutputMapReadOnly()
   {
      return new SensorOutputMapReadOnly()
      {

         @Override
         public long getVisionSensorTimestamp()
         {
            return 0;
         }

         @Override
         public long getTimestamp()
         {
            return 0;
         }

         @Override
         public long getSensorHeadPPSTimestamp()
         {
            return 0;
         }

         @Override
         public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
         {
            return false;
         }

         @Override
         public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
         {
            return 0;
         }

         @Override
         public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
         {
            return null;
         }

         @Override
         public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
         {
            return null;
         }
      };
   }

}
