package us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ContactBasedFootSwitch;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedContactBasedFootSwitch;

public class SDFQuadrupedPerfectSimulatedSensor extends SDFPerfectSimulatedSensorReader implements FootSwitchOutputReadOnly
{
   private final QuadrantDependentList<ContactBasedFootSwitch> footSwitches = new QuadrantDependentList<>();
   
   private final OneDoFJoint[] sensorOneDoFJoints;
   
   private final BooleanYoVariable enableDrives;
   
   public SDFQuadrupedPerfectSimulatedSensor(SDFRobot sdfRobot, SDFFullRobotModel sdfFullRobotModel, QuadrupedJointNameMap jointMap)
   {
      super(sdfRobot, sdfFullRobotModel, null);
      
      sensorOneDoFJoints = sdfFullRobotModel.getOneDoFJoints();
      
      //FootSwitches
      ArrayList<GroundContactPoint> groundContactPoints = sdfRobot.getAllGroundContactPoints();
      
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         String prefix = quadrant.getCamelCaseNameForStartOfExpression();
         String jointBeforeFootName = jointMap.getJointBeforeFootName(quadrant);
         Joint jointBeforeFoot = sdfRobot.getJoint(jointBeforeFootName);
         
         for(GroundContactPoint groundContactPoint : groundContactPoints)
         {
            if(groundContactPoint.getParentJoint() == jointBeforeFoot)
            {
               footSwitches.set(quadrant, new SimulatedContactBasedFootSwitch(prefix + groundContactPoint.getName(), groundContactPoint, super.getYoVariableRegistry()));
            }
         }
      }
      
      enableDrives = new BooleanYoVariable("enableDrives", getYoVariableRegistry());
   }
   
   @Override
   public void read()
   {
      for(int i = 0; i < sensorOneDoFJoints.length; i++)
      {
        sensorOneDoFJoints[i].setEnabled(enableDrives.getBooleanValue());
      }

      super.read();
   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return footSwitches.get(quadrant).isInContact();
   }
   
   @Override
   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
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
   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
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
   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
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
   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
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
   public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
   {
      for(int i = 0; i < sensorOneDoFJoints.length; i++)
      {
         if(sensorOneDoFJoints[i] == oneDoFJoint)
         {
            return sensorOneDoFJoints[i].isEnabled();
         }
      }
      return false;
   }
}
