package us.ihmc.sensorProcessing.simulatedSensors;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.IMUMount;
import com.yobotics.simulationconstructionset.KinematicPoint;
import com.yobotics.simulationconstructionset.OneDegreeOfFreedomJoint;
import com.yobotics.simulationconstructionset.Robot;

public class StateEstimatorSensorDefinitionsFromRobotFactory
{
   private final SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap;
   private final Robot robot;

   private final LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions;
   private final LinkedHashMap<KinematicPoint, PointVelocitySensorDefinition> pointVelocitySensorDefinitions;
   private final LinkedHashMap<KinematicPoint, PointPositionSensorDefinition> pointPositionSensorDefinitions;
   
   private final StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;

   public StateEstimatorSensorDefinitionsFromRobotFactory(SCSToInverseDynamicsJointMap scsToInverseDynamicsJointMap, Robot robot, double controlDT,
           ArrayList<IMUMount> imuMounts, ArrayList<KinematicPoint> positionPoints, ArrayList<KinematicPoint> velocityPoints)
   {
      this.scsToInverseDynamicsJointMap = scsToInverseDynamicsJointMap;
      this.robot = robot;

      this.imuDefinitions = generateIMUDefinitions(imuMounts);
      this.pointPositionSensorDefinitions = generatePointPositionSensorDefinitions(positionPoints);
      this.pointVelocitySensorDefinitions = generatePointVelocitySensorDefinitions(velocityPoints);

      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      createAndAddOneDoFPositionAndVelocitySensors();
      createAndAddOrientationSensors(imuDefinitions);
      createAndAddAngularVelocitySensors(imuDefinitions);
      createAndAddLinearAccelerationSensors(imuDefinitions);
      
      createAndAddPointPositionSensors(positionPoints);
      createAndAddPointVelocitySensors(velocityPoints);
   }
   
   public Map<IMUMount, IMUDefinition> getIMUDefinitions()
   {
      return imuDefinitions;
   }
   
   public Map<KinematicPoint, PointPositionSensorDefinition> getPointPositionSensorDefinitions()
   {
      return pointPositionSensorDefinitions;
   }
   
   public Map<KinematicPoint, PointVelocitySensorDefinition> getPointVelocitySensorDefinitions()
   {
      return pointVelocitySensorDefinitions;
   }

   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   private LinkedHashMap<IMUMount, IMUDefinition> generateIMUDefinitions(ArrayList<IMUMount> imuMounts)
   {
      LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions = new LinkedHashMap<IMUMount, IMUDefinition>();

      for (IMUMount imuMount : imuMounts)
      {
         RigidBody rigidBody = scsToInverseDynamicsJointMap.getRigidBody(imuMount.getParentJoint());
         Transform3D transformFromMountToJoint = new Transform3D();
         imuMount.getTransformFromMountToJoint(transformFromMountToJoint);
         IMUDefinition imuDefinition = new IMUDefinition(imuMount.getName(), rigidBody, transformFromMountToJoint);
         imuDefinitions.put(imuMount, imuDefinition);
      }

      return imuDefinitions;
   }

   private LinkedHashMap<KinematicPoint, PointPositionSensorDefinition> generatePointPositionSensorDefinitions(ArrayList<KinematicPoint> positionPoints)
   {
      LinkedHashMap<KinematicPoint, PointPositionSensorDefinition> pointPositionSensorDefinitions = new LinkedHashMap<KinematicPoint,
                                                                                                       PointPositionSensorDefinition>();

      for (KinematicPoint kinematicPoint : positionPoints)
      {
         RigidBody rigidBody = scsToInverseDynamicsJointMap.getRigidBody(kinematicPoint.getParentJoint());
         Vector3d offset = new Vector3d();
         kinematicPoint.getOffset(offset);

         PointPositionSensorDefinition pointPositionSensorDefinition = new PointPositionSensorDefinition(kinematicPoint.getName(), rigidBody, offset);
         pointPositionSensorDefinitions.put(kinematicPoint, pointPositionSensorDefinition);
      }

      return pointPositionSensorDefinitions;
   }
   
   private LinkedHashMap<KinematicPoint, PointVelocitySensorDefinition> generatePointVelocitySensorDefinitions(ArrayList<KinematicPoint> velocityPoints)
   {
      LinkedHashMap<KinematicPoint, PointVelocitySensorDefinition> pointVelocitySensorDefinitions = new LinkedHashMap<KinematicPoint,
                                                                                                       PointVelocitySensorDefinition>();

      for (KinematicPoint kinematicPoint : velocityPoints)
      {
         RigidBody rigidBody = scsToInverseDynamicsJointMap.getRigidBody(kinematicPoint.getParentJoint());
         Vector3d offset = new Vector3d();
         kinematicPoint.getOffset(offset);

         PointVelocitySensorDefinition pointVelocitySensorDefinition = new PointVelocitySensorDefinition(kinematicPoint.getName(), rigidBody, offset);
         pointVelocitySensorDefinitions.put(kinematicPoint, pointVelocitySensorDefinition);
      }

      return pointVelocitySensorDefinitions;
   }


   public void createAndAddOneDoFPositionAndVelocitySensors()
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = scsToInverseDynamicsJointMap.getInverseDynamicsOneDoFJoint(oneDegreeOfFreedomJoint);

         stateEstimatorSensorDefinitions.addJointPositionSensorDefinition(oneDoFJoint);
         stateEstimatorSensorDefinitions.addJointVelocitySensorDefinition(oneDoFJoint);
      }
   }

   public void createAndAddOrientationSensors(LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);
         stateEstimatorSensorDefinitions.addOrientationSensorDefinition(imuDefinition);
      }
   }

   public void createAndAddAngularVelocitySensors(LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);
         stateEstimatorSensorDefinitions.addAngularVelocitySensorDefinition(imuDefinition);
      }
   }


   public void createAndAddLinearAccelerationSensors(LinkedHashMap<IMUMount, IMUDefinition> imuDefinitions)
   {
      Set<IMUMount> imuMounts = imuDefinitions.keySet();

      for (IMUMount imuMount : imuMounts)
      {
         IMUDefinition imuDefinition = imuDefinitions.get(imuMount);

         stateEstimatorSensorDefinitions.addLinearAccelerationSensorDefinition(imuDefinition);
      }
   }
   
   public void createAndAddPointPositionSensors(ArrayList<KinematicPoint> positionPoints)
   {
      for (KinematicPoint kinematicPoint : positionPoints)
      {
         PointPositionSensorDefinition pointPositionSensorDefinition = pointPositionSensorDefinitions.get(kinematicPoint);
         stateEstimatorSensorDefinitions.addPointPositionSensorDefinition(pointPositionSensorDefinition);
      }
   }

   public void createAndAddPointVelocitySensors(ArrayList<KinematicPoint> velocityPoints)
   {
      for (KinematicPoint kinematicPoint : velocityPoints)
      {
         PointVelocitySensorDefinition pointVelocitySensorDefinition = pointVelocitySensorDefinitions.get(kinematicPoint);
         stateEstimatorSensorDefinitions.addPointVelocitySensorDefinition(pointVelocitySensorDefinition);
      }
   }
}
