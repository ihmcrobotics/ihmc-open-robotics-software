package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

public class JointDescription implements RobotDescriptionNode
{
   private String name;
   private final ArrayList<JointDescription> childrenJointDescriptions = new ArrayList<>();

   private JointDescription parentJoint;
   private Vector3d offsetFromParentJoint = new Vector3d();

   private LinkDescription link;

   private final ArrayList<KinematicPointDescription> kinematicPoints = new ArrayList<>();
   private final ArrayList<ExternalForcePointDescription> externalForcePoints = new ArrayList<>();
   private final ArrayList<GroundContactPointDescription> groundContactPoints = new ArrayList<>();

   private final ArrayList<JointWrenchSensorDescription> wrenchSensors = new ArrayList<>();
   private final ArrayList<CameraSensorDescription> cameraSensors = new ArrayList<>();
   private final ArrayList<IMUSensorDescription> imuSensors = new ArrayList<>();
   private final ArrayList<LidarSensorDescription> lidarSensors = new ArrayList<>();
   private final ArrayList<ForceSensorDescription> forceSensors = new ArrayList<>();

   private boolean isDynamic = true;

   public JointDescription(String name, Vector3d offsetFromParentJoint)
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

   public void setOffsetFromParentJoint(Vector3d offset)
   {
      this.offsetFromParentJoint.set(offset);
   }

   public JointDescription getParentJoint()
   {
      return parentJoint;
   }

   public void getOffsetFromParentJoint(Vector3d offsetToPack)
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
   
   
   public static void scaleChildrenJoint(ArrayList<JointDescription> childrenJoints, double factor, double massScalePower)
   {
      Vector3d offsetFromParentJoint = new Vector3d();
      for(int i = 0; i < childrenJoints.size(); i++)
      {
         JointDescription description = childrenJoints.get(i);

         description.getOffsetFromParentJoint(offsetFromParentJoint);
         offsetFromParentJoint.scale(factor);
         description.setOffsetFromParentJoint(offsetFromParentJoint);
         
         description.scale(factor, massScalePower);
      }

   }
   

   @Override
   public void scale(double factor, double massScalePower)
   {
      link.scale(factor, massScalePower);
      
      JointDescription.scaleChildrenJoint(getChildrenJoints(), factor, massScalePower);
   }
}
