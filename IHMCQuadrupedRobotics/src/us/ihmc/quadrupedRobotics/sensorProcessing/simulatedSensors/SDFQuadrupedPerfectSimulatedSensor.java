package us.ihmc.quadrupedRobotics.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.sensorProcessing.sensorProcessors.FootSwitchOutputReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.ContactBasedFootSwitch;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AuxiliaryRobotData;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.robotController.RawSensorReader;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedContactBasedFootSwitch;

public class SDFQuadrupedPerfectSimulatedSensor implements RawSensorReader, SensorOutputMapReadOnly, SensorRawOutputMapReadOnly, FootSwitchOutputReadOnly
{
   private final String name;
   private final SDFRobot sdfRobot;
   private final SixDoFJoint rootJoint;
   
   private final ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>> revoluteJoints = new ArrayList<ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>>();
   
   private final QuadrantDependentList<ContactBasedFootSwitch> footSwitches = new QuadrantDependentList<>();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final LongYoVariable timestamp = new LongYoVariable("timestamp", registry);
   private final LongYoVariable visionSensorTimestamp = new LongYoVariable("visionSensorTimestamp", registry);
   private final LongYoVariable sensorHeadPPSTimetamp = new LongYoVariable("sensorHeadPPSTimetamp", registry);
   
   
   public SDFQuadrupedPerfectSimulatedSensor(SDFRobot sdfRobot, SDFFullRobotModel sdfFullRobotModel, QuadrupedJointNameMap jointMap)
   {
      name = sdfFullRobotModel.getModelName() + "SimulatedSensorReader";
      rootJoint = sdfFullRobotModel.getRootJoint();
      this.sdfRobot = sdfRobot;
      
      InverseDynamicsJoint[] jointsArray = ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor());

      for (InverseDynamicsJoint joint : jointsArray)
      {
         if (joint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) joint;
            String name = oneDoFJoint.getName();
            OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = sdfRobot.getOneDegreeOfFreedomJoint(name);

            ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair = new ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint>(oneDegreeOfFreedomJoint, oneDoFJoint);
            revoluteJoints.add(jointPair);
         }
      }
      
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
               footSwitches.put(quadrant, new SimulatedContactBasedFootSwitch(prefix + groundContactPoint.getName(), groundContactPoint, registry));
            }
         }
      }
      
   }


   @Override
   public void initialize()
   {
      read();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public long getTimestamp()
   {
      return timestamp.getLongValue();
   }


   @Override
   public long getVisionSensorTimestamp()
   {
      return visionSensorTimestamp.getLongValue();
   }


   @Override
   public long getSensorHeadPPSTimestamp()
   {
      return sensorHeadPPSTimetamp.getLongValue();
   }


   @Override
   public double getJointPositionProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getQ();
   }

   @Override
   public double getJointVelocityProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getQd();
   }

   @Override
   public double getJointAccelerationProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getQdd();
   }

   @Override
   public double getJointTauProcessedOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getTau();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMUProcessedOutputs()
   {
      return new ArrayList<>();
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorProcessedOutputs()
   {
      return null;
   }

   @Override
   public double getJointPositionRawOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getQ();
   }

   @Override
   public double getJointVelocityRawOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getQd();
   }

   @Override
   public double getJointAccelerationRawOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getQdd();
   }

   @Override
   public double getJointTauRawOutput(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.getTau();
   }

   @Override
   public boolean isJointEnabled(OneDoFJoint oneDoFJoint)
   {
      return oneDoFJoint.isEnabled();
   }

   @Override
   public List<? extends IMUSensorReadOnly> getIMURawOutputs()
   {
      return new ArrayList<>();
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

   private final RigidBodyTransform temporaryRootToWorldTransform = new RigidBodyTransform();
   
   @Override
   public void read()
   {
      // Think about adding root body acceleration to the fullrobotmodel
      readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations();
      readAndUpdateRootJointPositionAndOrientation();
      updateReferenceFrames();
      readAndUpdateRootJointAngularAndLinearVelocity();
      
      long timestamp = TimeTools.secondsToNanoSeconds(sdfRobot.getTime());
      this.timestamp.set(timestamp);
      this.visionSensorTimestamp.set(timestamp);
      this.sensorHeadPPSTimetamp.set(timestamp);

   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return footSwitches.get(quadrant).isInContact();
   }


   private void readAndUpdateRootJointAngularAndLinearVelocity()
   {
      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame pelvisFrame = rootJoint.getFrameAfterJoint();

      FrameVector linearVelocity = sdfRobot.getRootJointVelocity();
      linearVelocity.changeFrame(pelvisFrame);

      FrameVector angularVelocity = sdfRobot.getPelvisAngularVelocityInPelvisFrame(pelvisFrame);
      angularVelocity.changeFrame(pelvisFrame);

      Twist bodyTwist = new Twist(pelvisFrame, elevatorFrame, pelvisFrame, linearVelocity.getVector(), angularVelocity.getVector());
      rootJoint.setJointTwist(bodyTwist);
   }

   private void updateReferenceFrames()
   {
      rootJoint.getPredecessor().updateFramesRecursively();
   }

   private void readAndUpdateRootJointPositionAndOrientation()
   {
      packRootTransform(sdfRobot, temporaryRootToWorldTransform);
      temporaryRootToWorldTransform.normalize();
      rootJoint.setPositionAndRotation(temporaryRootToWorldTransform);
   }

   private void readAndUpdateOneDoFJointPositionsVelocitiesAndAccelerations()
   {
      for (ImmutablePair<OneDegreeOfFreedomJoint, OneDoFJoint> jointPair : revoluteJoints)
      {
         OneDegreeOfFreedomJoint pinJoint = jointPair.getLeft();
         OneDoFJoint revoluteJoint = jointPair.getRight();

         revoluteJoint.setQ(pinJoint.getQ().getDoubleValue());
         revoluteJoint.setQd(pinJoint.getQD().getDoubleValue());
         revoluteJoint.setQdd(pinJoint.getQDD().getDoubleValue());
      }
   }
   
   protected void packRootTransform(SDFRobot robot, RigidBodyTransform transformToPack)
   {
      robot.getRootJointToWorldTransform(transformToPack);
   }
   
   
}
