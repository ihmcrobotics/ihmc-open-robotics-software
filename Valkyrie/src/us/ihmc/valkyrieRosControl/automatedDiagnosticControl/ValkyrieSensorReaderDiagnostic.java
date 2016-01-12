package us.ihmc.valkyrieRosControl.automatedDiagnosticControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.logging.Logger;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.rosControl.JointHandle;
import us.ihmc.rosControl.valkyrie.ForceTorqueSensorHandle;
import us.ihmc.rosControl.valkyrie.IMUHandle;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;
import us.ihmc.valkyrieRosControl.dataHolders.YoIMUHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoMicroStrainIMUHandleHolder;
import us.ihmc.valkyrieRosControl.dataHolders.YoSwitchableFilterModeIMUHandleHolder;

public class ValkyrieSensorReaderDiagnostic
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

   private final Logger logger = Logger.getLogger(getClass().getSimpleName());

   private final ArrayList<JointHandle> jointHandleList = new ArrayList<>();
   private final LinkedHashMap<JointHandle, OneDoFJoint> jointHandleToJointMap = new LinkedHashMap<>();

   private final ArrayList<YoIMUHandleHolder> imuHandleHolders = new ArrayList<>();

   private final ArrayList<ForceTorqueSensorHandle> forceTorqueSensorHandleList = new ArrayList<>();
   private final LinkedHashMap<ForceTorqueSensorHandle, ForceSensorDefinition> forceTorqueSensorHandleToDefinitionMap = new LinkedHashMap<>();

   private final TimestampProvider timestampProvider;

   public ValkyrieSensorReaderDiagnostic(TimestampProvider timestampProvider, StateEstimatorParameters stateEstimatorParameters)
   {
      this.timestampProvider = timestampProvider;
   }

   public void setupJointHandles(HashMap<String, JointHandle> jointHandles, InverseDynamicsJoint rootJoint)
   {
      // Planning on modifying the Map
      jointHandles = new HashMap<>(jointHandles);

      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()))
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
            JointHandle jointHandle = jointHandles.remove(joint.getName());
            if (jointHandle != null)
            {
               logger.info("Attaching " + jointHandle.getClass().getSimpleName() + ": " + jointHandle.getName() + "to " + joint.getClass().getSimpleName()
                     + ": " + joint.getName());
               jointHandleList.add(jointHandle);
               jointHandleToJointMap.put(jointHandle, oneDoFJoint);
            }
            else
            {
               logger.warning("Could not find a JointHandle for the " + joint.getClass().getSimpleName() + ": " + joint.getName());
            }
         }
      }

      if (!jointHandles.isEmpty())
      {
         for (String jointHandleName : jointHandles.keySet())
         {
            logger.warning("Could not find a joint for the " + jointHandles.get(jointHandleName).getClass().getSimpleName() + ": " + jointHandleName);
         }
      }

      if (jointHandleList.isEmpty())
         logger.severe("Could not find any joint handle to setup, diagnostic will be limited.");
   }

   public void setupIMUHandles(HashMap<String, IMUHandle> imuHandles, IMUDefinition[] imuDefinitions, HashMap<String, Integer> imuUSBSerialIds)
   {
      // Planning on modifying these Maps
      imuHandles = new HashMap<>(imuHandles);
      imuUSBSerialIds = new HashMap<>(imuUSBSerialIds);

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         if (ValkyrieRosControlController.USE_USB_MICROSTRAIN_IMUS)
         {
            logger.info("Only looking for IMUs that are connected through the IHMC UDP server.");

            Integer imuUSBSerialId = imuUSBSerialIds.remove(imuDefinition.getName());
            stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
            if (imuUSBSerialId != null)
            {
               YoMicroStrainIMUHandleHolder holder = YoMicroStrainIMUHandleHolder.create(imuUSBSerialId, imuDefinition, registry);
               imuHandleHolders.add(holder);
               logger.info("Creating " + YoMicroStrainIMUHandleHolder.class.getName() + " associated with the " + imuDefinition.getClass().getSimpleName()
                     + ": " + imuDefinition.getName() + ", serial ID: " + imuUSBSerialId);
            }
            else
            {
               logger.warning("Cannot create listener for IMU " + imuDefinition.getName() + ", cannot find corresponding serial.");
            }

            if (!imuUSBSerialIds.isEmpty())
            {
               for (String imuName : imuUSBSerialIds.keySet())
               {
                  logger.warning("Could not find matching " + IMUDefinition.class.getSimpleName() + " for the IMU " + imuName + " with the serial ID: "
                        + imuUSBSerialIds.get(imuName));
               }
            }
         }
         else if (ValkyrieRosControlController.USE_SWITCHABLE_FILTER_HOLDER_FOR_NON_USB_IMUS)
         {
            logger.info(
                  "Only looking for twin IMUs (2 software IMUs per physical IMU to enable switching the two MicroStrain filters) that are connected through the JSC server.");

            String name = imuDefinition.getName();
            name = name.replace(imuDefinition.getRigidBody().getName() + "_", "");

            IMUHandle complimentaryFilterHandle = imuHandles.remove("CF" + name);
            IMUHandle kalmanFilterHandle = imuHandles.remove("EF" + name);

            if (complimentaryFilterHandle != null && kalmanFilterHandle != null)
            {
               YoSwitchableFilterModeIMUHandleHolder holder = YoSwitchableFilterModeIMUHandleHolder.create(complimentaryFilterHandle, kalmanFilterHandle,
                     imuDefinition, registry);
               imuHandleHolders.add(holder);
               stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
               logger.info("Attaching " + complimentaryFilterHandle.getName() + " (IMU running the complementary filter) and " + kalmanFilterHandle.getName()
                     + " (IMU running the kalman filter) to " + imuDefinition.getClass().getSimpleName() + ": " + imuDefinition.getName());
            }
            else
            {
               logger.warning("Could not find a twin IMU handle for " + imuDefinition.getClass().getName() + ": " + imuDefinition.getName());
            }
         }
         else
         {
            logger.info("Only looking for IMUs that are connected through the JSC server.");

            String name = imuDefinition.getName();
            name = name.replace(imuDefinition.getRigidBody().getName() + "_", "");
            IMUHandle imuHandle = imuHandles.remove(name);

            if (imuHandle != null)
            {
               stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
               YoIMUHandleHolder holder = new YoIMUHandleHolder(imuHandle, imuDefinition, registry);
               imuHandleHolders.add(holder);
               logger.info("Attaching " + imuHandle.getClass().getSimpleName() + " to the " + imuDefinition.getClass().getSimpleName() + ": " + imuDefinition.getName());
            }
            else
            {
               logger.warning("Could not find a IMU handle for " + imuDefinition.getClass().getName() + ": " + imuDefinition.getName());
            }
         }
      }

      if (!ValkyrieRosControlController.USE_USB_MICROSTRAIN_IMUS)
      {
         if (!imuHandles.isEmpty())
         {
            for (String imuName : imuHandles.keySet())
            {
               logger.warning("Could not find matching " + IMUDefinition.class.getSimpleName() + " for the " + imuHandles.get(imuName).getClass().getSimpleName() + ": " + imuName);
            }
         }
      }

      if (imuHandleHolders.isEmpty())
         logger.severe("Could not find any IMU handle to setup, diagnostic will be limited.");
   }

   public void setupForceTorqueSensorHandles(HashMap<String, ForceTorqueSensorHandle> forceTorqueSensorHandles, ForceSensorDefinition[] forceSensorDefinitions)
   {
      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         ForceTorqueSensorHandle forceTorqueSensorHandle = forceTorqueSensorHandles.remove(forceSensorDefinition.getSensorName());
         if (forceTorqueSensorHandle != null)
         {
            stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
            forceTorqueSensorHandleList.add(forceTorqueSensorHandle);
            forceTorqueSensorHandleToDefinitionMap.put(forceTorqueSensorHandle, forceSensorDefinition);
            logger.info("Attaching " + forceTorqueSensorHandle.getClass().getSimpleName() + " to the " + forceSensorDefinition.getClass().getSimpleName() + ": " + forceSensorDefinition.getSensorName());
         }
         else
         {
            logger.warning("Could not find a force torque sensor handle for " + forceSensorDefinition.getClass().getName() + ": " + forceSensorDefinition.getSensorName());
         }
      }

      if (forceTorqueSensorHandleList.isEmpty())
         logger.severe("Could not find any force torque sensor handle to setup, diagnostic will be limited.");
   }
}
