package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class JointDescription implements RobotDescriptionNode
{
   private String name;
   private final ArrayList<JointDescription> childrenJointDescriptions = new ArrayList<>();

   private JointDescription parentJoint;
   private Vector3D offsetFromParentJoint = new Vector3D();

   private LinkDescription link;

   // Lists of kinematic points on the robot. When adding types of kinematic points, make sure to update the getAllKinematicPoints(List<KinematicPointDescription>) function
   private final ArrayList<KinematicPointDescription> kinematicPoints = new ArrayList<>();
   private final ArrayList<ExternalForcePointDescription> externalForcePoints = new ArrayList<>();
   private final ArrayList<GroundContactPointDescription> groundContactPoints = new ArrayList<>();

   // Lists of sensors. When adding sensors, make sure to update the getSensors(List<SensorDescription>) function.
   private final ArrayList<JointWrenchSensorDescription> wrenchSensors = new ArrayList<>();
   private final ArrayList<CameraSensorDescription> cameraSensors = new ArrayList<>();
   private final ArrayList<IMUSensorDescription> imuSensors = new ArrayList<>();
   private final ArrayList<LidarSensorDescription> lidarSensors = new ArrayList<>();
   private final ArrayList<ForceSensorDescription> forceSensors = new ArrayList<>();

   private boolean isDynamic = true;

   public JointDescription(String name, Vector3D offsetFromParentJoint)
   {
      this.name = name;
      this.offsetFromParentJoint.set(offsetFromParentJoint);
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setParentJoint(JointDescription parentJoint)
   {
      this.parentJoint = parentJoint;
   }

   public void setOffsetFromParentJoint(Vector3D offset)
   {
      this.offsetFromParentJoint.set(offset);
   }

   public JointDescription getParentJoint()
   {
      return parentJoint;
   }

   public void getOffsetFromParentJoint(Vector3D offsetToPack)
   {
      offsetToPack.set(offsetFromParentJoint);
   }

   public LinkDescription getLink()
   {
      return link;
   }

   public void setLink(LinkDescription link)
   {
      this.link = link;
   }

   public void addJoint(JointDescription childJointDescription)
   {
      childrenJointDescriptions.add(childJointDescription);

      if (childJointDescription.getParentJoint() != null)
      {
         throw new RuntimeException("JointDescription " + childJointDescription.getName() + "already has a parent joint: " + childJointDescription.getParentJoint().getName());
      }

      childJointDescription.setParentJoint(this);
   }
   
   public boolean removeJoint(JointDescription childJointDescription)
   {
      return childrenJointDescriptions.remove(childJointDescription);
   }

   @Override
   public ArrayList<JointDescription> getChildrenJoints()
   {
      return childrenJointDescriptions;
   }

   public void addGroundContactPoint(GroundContactPointDescription groundContactPointDescription)
   {
      this.groundContactPoints.add(groundContactPointDescription);
   }

   public ArrayList<GroundContactPointDescription> getGroundContactPoints()
   {
      return groundContactPoints;
   }

   public void addExternalForcePoint(ExternalForcePointDescription externalForcePointDescription)
   {
      this.externalForcePoints.add(externalForcePointDescription);
   }

   public ArrayList<ExternalForcePointDescription> getExternalForcePoints()
   {
      return externalForcePoints;
   }

   public void addKinematicPoint(KinematicPointDescription kinematicPointDescription)
   {
      this.kinematicPoints.add(kinematicPointDescription);
   }

   public ArrayList<KinematicPointDescription> getKinematicPoints()
   {
      return kinematicPoints;
   }

   public void addJointWrenchSensor(JointWrenchSensorDescription jointWrenchSensorDescription)
   {
      wrenchSensors.add(jointWrenchSensorDescription);
   }

   public ArrayList<JointWrenchSensorDescription> getWrenchSensors()
   {
      return wrenchSensors;
   }

   public void addCameraSensor(CameraSensorDescription cameraSensorDescription)
   {
      cameraSensors.add(cameraSensorDescription);
   }

   public ArrayList<CameraSensorDescription> getCameraSensors()
   {
      return cameraSensors;
   }

   public void addIMUSensor(IMUSensorDescription imuSensorDescription)
   {
      imuSensors.add(imuSensorDescription);
   }

   public ArrayList<IMUSensorDescription> getIMUSensors()
   {
      return imuSensors;
   }

   public void addLidarSensor(LidarSensorDescription lidarSensor)
   {
      lidarSensors.add(lidarSensor);
   }

   public ArrayList<LidarSensorDescription> getLidarSensors()
   {
      return lidarSensors;
   }

   public void addForceSensor(ForceSensorDescription forceSensor)
   {
      forceSensors.add(forceSensor);
   }

   public ArrayList<ForceSensorDescription> getForceSensors()
   {
      return forceSensors;
   }

   public void setIsDynamic(boolean isDynamic)
   {
      this.isDynamic = isDynamic;
   }

   public boolean isDynamic()
   {
      return isDynamic;
   }
   
   public void getSensors(List<SensorDescription> sensors)
   {
      sensors.addAll(wrenchSensors);
      sensors.addAll(cameraSensors);
      sensors.addAll(imuSensors);
      sensors.addAll(lidarSensors);
      sensors.addAll(forceSensors);
   }
   
   public void getAllKinematicPoints(List<KinematicPointDescription> allKinematicPoints)
   {
      allKinematicPoints.addAll(this.kinematicPoints);
      allKinematicPoints.addAll(this.externalForcePoints);
      allKinematicPoints.addAll(this.groundContactPoints);
   }
   
   
   public static void scaleChildrenJoint(ArrayList<JointDescription> childrenJoints, double factor, double massScalePower, List<String>  ignoreInertiaScaleJointList)
   {
      Vector3D offsetFromParentJoint = new Vector3D();
      for(int i = 0; i < childrenJoints.size(); i++)
      {
         JointDescription description = childrenJoints.get(i);

         description.getOffsetFromParentJoint(offsetFromParentJoint);
         offsetFromParentJoint.scale(factor);
         description.setOffsetFromParentJoint(offsetFromParentJoint);
         
         description.scale(factor, massScalePower, ignoreInertiaScaleJointList);
      }

   }
   

   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      scaleSensorsOffsets(factor);
      scaleAllKinematicsPointOffsets(factor);
      
      boolean scaleInertia = true;
      if(ignoreInertiaScaleJointList.contains(getName()))
      {
         scaleInertia = false;
      }
      link.scale(factor, massScalePower, scaleInertia);
      JointDescription.scaleChildrenJoint(getChildrenJoints(), factor, massScalePower, ignoreInertiaScaleJointList);
   }

   private void scaleSensorsOffsets(double factor)
   {
      ArrayList<SensorDescription> sensors = new ArrayList<>();
      getSensors(sensors);
      
      for(int i = 0; i < sensors.size(); i++)
      {
         SensorDescription sensor = sensors.get(i);
         RigidBodyTransform transformToJoint = sensor.getTransformToJoint();
         Vector3D translation = new Vector3D();
         transformToJoint.getTranslation(translation);
         translation.scale(factor);
         transformToJoint.setTranslation(translation);
         sensor.setTransformToJoint(transformToJoint);
         
      }
   }
   
   private void scaleAllKinematicsPointOffsets(double factor)
   {
      ArrayList<KinematicPointDescription> allKinematicPoints = new ArrayList<>();
      getAllKinematicPoints(allKinematicPoints);
      for(int i = 0; i < allKinematicPoints.size(); i++)
      {
         KinematicPointDescription kinematicPoint = allKinematicPoints.get(i);
         
         Vector3D offset = kinematicPoint.getOffsetFromJoint();
         offset.scale(factor);
         kinematicPoint.setOffsetFromJoint(offset);
      }
            
   }
}
