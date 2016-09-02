package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.HashMap;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.robotDescription.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.ExternalForcePointDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.FloatingPlanarJointDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.JointWrenchSensorDescription;
import us.ihmc.robotics.robotDescription.KinematicPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;

public class RobotConstructorFromRobotDescription
{
   private Robot robot;

   private HashMap<String, Joint> jointNameMap = new HashMap<>();
   private HashMap<JointDescription, Joint> jointDescriptionMap = new HashMap<>();

   private HashMap<String, CameraMount> cameraNameMap = new HashMap<>();
   private HashMap<CameraSensorDescription, CameraMount> cameraDescriptionMap = new HashMap<>();

   private HashMap<String, IMUMount> imuNameMap = new HashMap<>();
   private HashMap<IMUSensorDescription, IMUMount> imuDescriptionMap = new HashMap<>();

   private HashMap<String, JointWrenchSensor> wrenchSensorNameMap = new HashMap<>();
   private HashMap<JointWrenchSensorDescription, JointWrenchSensor> wrenchSensorDescriptionMap = new HashMap<>();

   public RobotConstructorFromRobotDescription(RobotDescription description)
   {
      constructRobotFromDescription(description);
   }

   public Robot getRobot()
   {
      return robot;
   }

   public Joint getJoint(String jointName)
   {
      return jointNameMap.get(jointName);
   }

   public Joint getJoint(JointDescription jointDescription)
   {
      return jointDescriptionMap.get(jointDescription);
   }

   public CameraMount getCameraMount(String cameraName)
   {
      return cameraNameMap.get(cameraName);
   }

   public CameraMount getCameraMount(CameraSensorDescription cameraSensorDescription)
   {
      return cameraDescriptionMap.get(cameraSensorDescription);
   }

   public IMUMount getIMUMount(String name)
   {
      return imuNameMap.get(name);
   }

   public IMUMount getIMUMount(IMUSensorDescription imuSensorDescription)
   {
      return imuDescriptionMap.get(imuSensorDescription);
   }

   public JointWrenchSensor getJointWrenchSensor(String name)
   {
      return wrenchSensorNameMap.get(name);
   }

   public JointWrenchSensor getJointWrenchSensor(JointWrenchSensorDescription jointWrenchSensorDescription)
   {
      return wrenchSensorDescriptionMap.get(jointWrenchSensorDescription);
   }

   private void constructRobotFromDescription(RobotDescription description)
   {
      robot = new Robot(description.getName());

      ArrayList<JointDescription> rootJointDescriptions = description.getRootJoints();

      for (JointDescription rootJointDescription : rootJointDescriptions)
      {
         Joint rootJoint = constructJointRecursively(rootJointDescription);
         robot.addRootJoint(rootJoint);
      }
   }

   private Joint constructJointRecursively(JointDescription jointDescription)
   {
      Joint joint = createSingleJoint(jointDescription);

      addGroundContactPoints(jointDescription, joint);
      addExternalForcePoints(jointDescription, joint);
      addKinematicPoints(jointDescription, joint);

      addCameraMounts(jointDescription, joint);
      addIMUMounts(jointDescription, joint);
      addJointWrenchSensors(jointDescription, joint);

      // Iterate over the children
      ArrayList<JointDescription> childrenJoints = jointDescription.getChildrenJoints();
      for (JointDescription childJointDescription : childrenJoints)
      {
         Joint childJoint = constructJointRecursively(childJointDescription);
         joint.addJoint(childJoint);
      }

      jointNameMap.put(joint.getName(), joint);
      jointDescriptionMap.put(jointDescription, joint);
      return joint;
   }

   private void addCameraMounts(JointDescription jointDescription, Joint joint)
   {
      ArrayList<CameraSensorDescription> cameraSensorDescriptions = jointDescription.getCameraSensors();
      for (CameraSensorDescription cameraSensorDescription : cameraSensorDescriptions)
      {
         CameraMount cameraMount = new CameraMount(cameraSensorDescription.getName(), cameraSensorDescription.getTransformToJoint(), robot);
         joint.addCameraMount(cameraMount);

         cameraNameMap.put(cameraMount.getName(), cameraMount);
         cameraDescriptionMap.put(cameraSensorDescription, cameraMount);
      }
   }

   private void addIMUMounts(JointDescription jointDescription, Joint joint)
   {
      ArrayList<IMUSensorDescription> imuSensorDescriptions = jointDescription.getIMUSensors();
      for (IMUSensorDescription imuSensorDescription : imuSensorDescriptions)
      {
         IMUMount imuMount = new IMUMount(imuSensorDescription.getName(), imuSensorDescription.getTransformToJoint(), robot);
         joint.addIMUMount(imuMount);

         imuNameMap.put(imuMount.getName(), imuMount);
         imuDescriptionMap.put(imuSensorDescription, imuMount);
      }
   }

   private void addJointWrenchSensors(JointDescription jointDescription, Joint joint)
   {
      ArrayList<JointWrenchSensorDescription> jointWrenchSensorDescriptions = jointDescription.getWrenchSensors();
      for (JointWrenchSensorDescription jointWrenchSensorDescription : jointWrenchSensorDescriptions)
      {
         JointWrenchSensor jointWrenchSensor = new JointWrenchSensor(jointWrenchSensorDescription.getName(), jointWrenchSensorDescription.getOffsetFromJoint(), robot);
         joint.addJointWrenchSensor(jointWrenchSensor);

         wrenchSensorNameMap.put(jointWrenchSensor.getName(), jointWrenchSensor);
         wrenchSensorDescriptionMap.put(jointWrenchSensorDescription, jointWrenchSensor);
      }
   }

   private void addGroundContactPoints(JointDescription jointDescription, Joint joint)
   {
      ArrayList<GroundContactPointDescription> groundContactPointDescriptions = jointDescription.getGroundContactPoints();

      for (GroundContactPointDescription groundContactPointDescription : groundContactPointDescriptions)
      {
         GroundContactPoint groundContactPoint = new GroundContactPoint(groundContactPointDescription.getName(), groundContactPointDescription.getOffsetFromJoint(), robot);
         joint.addGroundContactPoint(groundContactPoint);
      }
   }

   private void addExternalForcePoints(JointDescription jointDescription, Joint joint)
   {
      ArrayList<ExternalForcePointDescription> ExternalForcePointDescriptions = jointDescription.getExternalForcePoints();

      for (ExternalForcePointDescription ExternalForcePointDescription : ExternalForcePointDescriptions)
      {
         ExternalForcePoint ExternalForcePoint = new ExternalForcePoint(ExternalForcePointDescription.getName(), ExternalForcePointDescription.getOffsetFromJoint(), robot);
         joint.addExternalForcePoint(ExternalForcePoint);
      }
   }

   private void addKinematicPoints(JointDescription jointDescription, Joint joint)
   {
      ArrayList<KinematicPointDescription> KinematicPointDescriptions = jointDescription.getKinematicPoints();

      for (KinematicPointDescription KinematicPointDescription : KinematicPointDescriptions)
      {
         KinematicPoint KinematicPoint = new KinematicPoint(KinematicPointDescription.getName(), KinematicPointDescription.getOffsetFromJoint(), robot);
         joint.addKinematicPoint(KinematicPoint);
      }
   }

   private Joint createSingleJoint(JointDescription jointDescription)
   {
      Joint joint;

      if (jointDescription instanceof FloatingJointDescription)
      {
         FloatingJointDescription floatingJointDescription = (FloatingJointDescription) jointDescription;

         Vector3d offset = new Vector3d();
         floatingJointDescription.getOffsetFromParentJoint(offset);
         
         joint = new FloatingJoint(jointDescription.getName(), offset, robot);
      }
      
      else if (jointDescription instanceof FloatingPlanarJointDescription)
      {
         FloatingPlanarJointDescription floatingPlanarJointDescription = (FloatingPlanarJointDescription) jointDescription;

         joint = new FloatingPlanarJoint(jointDescription.getName(), robot, floatingPlanarJointDescription.getPlane());
      }

      else if (jointDescription instanceof PinJointDescription)
      {
         PinJointDescription pinJointDescription = (PinJointDescription) jointDescription;
         Vector3d offset = new Vector3d();
         pinJointDescription.getOffsetFromParentJoint(offset);

         joint = new PinJoint(jointDescription.getName(), offset, robot, pinJointDescription.getJointAxis());

         PinJoint pinJoint = (PinJoint) joint;

         if (pinJointDescription.containsLimitStops())
         {
            double[] limitStopParameters = pinJointDescription.getLimitStopParameters();

            double qMin = limitStopParameters[0];
            double qMax = limitStopParameters[1];
            double kLimit = limitStopParameters[2];
            double bLimit = limitStopParameters[3];

            pinJoint.setLimitStops(qMin, qMax, kLimit, bLimit);
         }
      }
      else if (jointDescription instanceof SliderJointDescription)
      {
         SliderJointDescription sliderJointDescription = (SliderJointDescription) jointDescription;
         Vector3d offset = new Vector3d();
         sliderJointDescription.getOffsetFromParentJoint(offset);

         joint = new SliderJoint(jointDescription.getName(), offset, robot, sliderJointDescription.getJointAxis());

         SliderJoint sliderJoint = (SliderJoint) joint;

         if (sliderJointDescription.containsLimitStops())
         {
            double[] limitStopParameters = sliderJointDescription.getLimitStopParameters();

            double qMin = limitStopParameters[0];
            double qMax = limitStopParameters[1];
            double kLimit = limitStopParameters[2];
            double bLimit = limitStopParameters[3];

            sliderJoint.setLimitStops(qMin, qMax, kLimit, bLimit);
         }
      }
      else
      {
         throw new RuntimeException("Don't support that joint type yet. Please implement it! Type = " + jointDescription.getClass());
      }

      Link link = createLink(jointDescription.getLink());
      joint.setLink(link);
      return joint;
   }

   private Link createLink(LinkDescription linkDescription)
   {
      Link link = new Link(linkDescription.getName());

      link.setMass(linkDescription.getMass());
      link.setComOffset(linkDescription.getCenterOfMassOffset());
      link.setMomentOfInertia(linkDescription.getMomentOfInertia());

      LinkGraphicsDescription linkGraphics = linkDescription.getLinkGraphics();
      link.setLinkGraphics(linkGraphics);

      return link;

   }
}
