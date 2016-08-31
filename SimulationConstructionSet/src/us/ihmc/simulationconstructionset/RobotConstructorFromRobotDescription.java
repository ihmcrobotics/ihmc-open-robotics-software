package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.robotDescription.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.ExternalForcePointDescription;
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

public class RobotConstructorFromRobotDescription
{
   private Robot robot;

   public RobotConstructorFromRobotDescription()
   {
   }

   public Robot constructRobotFromDescription(RobotDescription description)
   {
      robot = new Robot(description.getName());

      ArrayList<JointDescription> rootJointDescriptions = description.getRootJoints();

      for (JointDescription rootJointDescription : rootJointDescriptions)
      {
         Joint rootJoint = constructJointRecursively(rootJointDescription);
         robot.addRootJoint(rootJoint);
      }

      return robot;
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

      return joint;
   }

   private void addCameraMounts(JointDescription jointDescription, Joint joint)
   {
      ArrayList<CameraSensorDescription> cameraSensorDescriptions = jointDescription.getCameraSensors();
      for (CameraSensorDescription cameraSensorDescription : cameraSensorDescriptions)
      {
         CameraMount cameraMount = new CameraMount(cameraSensorDescription.getName(), cameraSensorDescription.getTransformToJoint(), robot);
         joint.addCameraMount(cameraMount);
      }
   }

   private void addIMUMounts(JointDescription jointDescription, Joint joint)
   {
      ArrayList<IMUSensorDescription> IMUSensorDescriptions = jointDescription.getIMUSensors();
      for (IMUSensorDescription IMUSensorDescription : IMUSensorDescriptions)
      {
         IMUMount IMUMount = new IMUMount(IMUSensorDescription.getName(), IMUSensorDescription.getTransformToJoint(), robot);
         joint.addIMUMount(IMUMount);
      }
   }

   private void addJointWrenchSensors(JointDescription jointDescription, Joint joint)
   {
      ArrayList<JointWrenchSensorDescription> jointWrenchSensorDescriptions = jointDescription.getWrenchSensors();
      for (JointWrenchSensorDescription jointWrenchSensorDescription : jointWrenchSensorDescriptions)
      {
         JointWrenchSensor jointWrenchSensor = new JointWrenchSensor(jointWrenchSensorDescription.getName(), jointWrenchSensorDescription.getOffsetFromJoint(), robot);
         joint.addJointWrenchSensor(jointWrenchSensor);
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

      if (jointDescription instanceof FloatingPlanarJointDescription)
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
      else
      {
         throw new RuntimeException("Don't support that joint type yet. Please implement it!");
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
