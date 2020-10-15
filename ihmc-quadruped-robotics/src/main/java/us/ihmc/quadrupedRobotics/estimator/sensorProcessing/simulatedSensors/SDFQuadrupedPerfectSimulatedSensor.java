package us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.ContactBasedFootSwitch;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.CommonQuadrupedReferenceFrames;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.simulationConstructionSetTools.simulatedSensors.SimulatedContactBasedFootSwitch;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoVariables.variable.YoBoolean;

public class SDFQuadrupedPerfectSimulatedSensor extends SDFPerfectSimulatedSensorReader implements FootSwitchOutputReadOnly, SensorReader
{
   private final QuadrantDependentList<ContactBasedFootSwitch> footSwitches = new QuadrantDependentList<>();

   private final List<OneDoFJointStateReadOnly> jointSensorOutputList = new ArrayList<>();
   private final Map<String, OneDoFJointStateReadOnly> jointNameToJointSensorOutputMap = new HashMap<>();

   private final YoBoolean enableDrives;

   private final QuadrantDependentList<FootSwitchInterface> otherFootSwitches;

   public SDFQuadrupedPerfectSimulatedSensor(FloatingRootJointRobot sdfRobot, FullQuadrupedRobotModel fullRobotModel,
                                             CommonQuadrupedReferenceFrames referenceFrames, QuadrantDependentList<FootSwitchInterface> footSwitches)
   {
      this(RobotQuadrant.values, sdfRobot, fullRobotModel, referenceFrames, footSwitches);
   }

   public SDFQuadrupedPerfectSimulatedSensor(RobotQuadrant[] quadrants, FloatingRootJointRobot sdfRobot, FullQuadrupedRobotModel fullRobotModel,
                                             CommonQuadrupedReferenceFrames referenceFrames, QuadrantDependentList<FootSwitchInterface> otherFootSwitches)
   {
      super(sdfRobot, fullRobotModel, referenceFrames);

      this.otherFootSwitches = otherFootSwitches;

      for (OneDoFJointBasics joint : fullRobotModel.getOneDoFJoints())
      {
         OneDoFJointStateReadOnly jointSensorOutput = OneDoFJointStateReadOnly.createFromOneDoFJoint(joint, true);
         jointSensorOutputList.add(jointSensorOutput);
         jointNameToJointSensorOutputMap.put(joint.getName(), jointSensorOutput);
      }

      //FootSwitches
      List<GroundContactPoint> groundContactPoints = sdfRobot.getAllGroundContactPoints();

      for (RobotQuadrant quadrant : quadrants)
      {
         String prefix = quadrant.getCamelCaseNameForStartOfExpression();
         JointBasics jointBeforeFoot = fullRobotModel.getFoot(quadrant).getParentJoint();

         for (GroundContactPoint groundContactPoint : groundContactPoints)
         {
            if (groundContactPoint.getParentJoint().getName().equals(jointBeforeFoot.getName()))
            {
               footSwitches.set(quadrant,
                                new SimulatedContactBasedFootSwitch(prefix + groundContactPoint.getName(), groundContactPoint, super.getYoRegistry()));
            }
         }
      }

      enableDrives = new YoBoolean("enableDrives", getYoRegistry());
      enableDrives.set(true);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public long read(SensorDataContext sensorDataContextToSet)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         otherFootSwitches.get(robotQuadrant).updateMeasurement();

      super.read();
      return getMonotonicTime();
   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return footSwitches.get(quadrant).isInContact();
   }

   @Override
   public OneDoFJointStateReadOnly getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint)
   {
      return jointNameToJointSensorOutputMap.get(oneDoFJoint.getName());
   }

   @Override
   public List<? extends OneDoFJointStateReadOnly> getOneDoFJointOutputs()
   {
      return jointSensorOutputList;
   }

   @Override
   public SensorOutputMapReadOnly getProcessedSensorOutputMap()
   {
      return this;
   }

   @Override
   public SensorOutputMapReadOnly getRawSensorOutputMap()
   {
      return this;
   }
}
