package us.ihmc.quadrupedRobotics.estimator.sensorProcessing.simulatedSensors;

import java.util.ArrayList;

import controller_msgs.msg.dds.AtlasAuxiliaryRobotData;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.estimator.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.ContactBasedFootSwitch;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.sensorProcessing.frames.CommonQuadrupedReferenceFrames;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.simulationConstructionSetTools.simulatedSensors.SimulatedContactBasedFootSwitch;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.yoVariables.variable.YoBoolean;

public class SDFQuadrupedPerfectSimulatedSensor extends SDFPerfectSimulatedSensorReader implements FootSwitchOutputReadOnly, SensorReader
{
   private final QuadrantDependentList<ContactBasedFootSwitch> footSwitches = new QuadrantDependentList<>();

   private final OneDoFJointBasics[] sensorOneDoFJoints;

   private final YoBoolean enableDrives;

   private final SDFPerfectSimulatedSensorReader sdfPerfectSimulatedSensorReader;
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

      sensorOneDoFJoints = fullRobotModel.getOneDoFJoints();

      //FootSwitches
      ArrayList<GroundContactPoint> groundContactPoints = sdfRobot.getAllGroundContactPoints();

      for(RobotQuadrant quadrant : quadrants)
      {
         String prefix = quadrant.getCamelCaseNameForStartOfExpression();
         JointBasics jointBeforeFoot = fullRobotModel.getFoot(quadrant).getParentJoint();

         for(GroundContactPoint groundContactPoint : groundContactPoints)
         {
            if(groundContactPoint.getParentJoint().getName().equals(jointBeforeFoot.getName()))
            {
               footSwitches.set(quadrant, new SimulatedContactBasedFootSwitch(prefix + groundContactPoint.getName(), groundContactPoint, super.getYoVariableRegistry()));
            }
         }
      }

      enableDrives = new YoBoolean("enableDrives", getYoVariableRegistry());
      enableDrives.set(true);

      sdfPerfectSimulatedSensorReader = new SDFPerfectSimulatedSensorReader(sdfRobot, fullRobotModel, referenceFrames);
   }

   @Override
   public void read()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         otherFootSwitches.get(robotQuadrant).updateMeasurement();

      for(int i = 0; i < sensorOneDoFJoints.length; i++)
      {
         // FIXME
//        sensorOneDoFJoints[i].setEnabled(enableDrives.getBooleanValue());
      }

      super.read();
   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return footSwitches.get(quadrant).isInContact();
   }

   @Override
   public double getJointPositionProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      for(int i = 0; i < sensorOneDoFJoints.length; i++)
      {
         if(sensorOneDoFJoints[i].getName() == oneDoFJoint.getName())
         {
            return sensorOneDoFJoints[i].getQ();
         }
      }
      return 0.0;
   }

   @Override
   public double getJointVelocityProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      for(int i = 0; i < sensorOneDoFJoints.length; i++)
      {
         if(sensorOneDoFJoints[i].getName() == oneDoFJoint.getName())
         {
            return sensorOneDoFJoints[i].getQd();
         }
      }
      return 0.0;
   }

   @Override
   public double getJointAccelerationProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      for(int i = 0; i < sensorOneDoFJoints.length; i++)
      {
         if(sensorOneDoFJoints[i].getName() == oneDoFJoint.getName())
         {
            return sensorOneDoFJoints[i].getQdd();
         }
      }
      return 0.0;
   }

   @Override
   public double getJointTauProcessedOutput(OneDoFJointBasics oneDoFJoint)
   {
      for(int i = 0; i < sensorOneDoFJoints.length; i++)
      {
         if(sensorOneDoFJoints[i].getName() == oneDoFJoint.getName())
         {
            return sensorOneDoFJoints[i].getTau();
         }
      }
      return 0.0;
   }

   @Override
   public boolean isJointEnabled(OneDoFJointBasics oneDoFJoint)
   {
      for(int i = 0; i < sensorOneDoFJoints.length; i++)
      {
         if(sensorOneDoFJoints[i] == oneDoFJoint)
         {
            return true;
            // FIXME
//            return sensorOneDoFJoints[i].isEnabled();
         }
      }
      return false;
   }

   @Override
   public AtlasAuxiliaryRobotData newAuxiliaryRobotDataInstance()
   {
      return null;
   }

   @Override
   public SensorOutputMapReadOnly getSensorOutputMapReadOnly()
   {
      return sdfPerfectSimulatedSensorReader;
   }

   @Override
   public SensorRawOutputMapReadOnly getSensorRawOutputMapReadOnly()
   {
      return sdfPerfectSimulatedSensorReader;
   }
}
