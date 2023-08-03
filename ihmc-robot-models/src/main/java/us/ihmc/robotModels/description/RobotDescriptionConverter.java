package us.ihmc.robotModels.description;

import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.robotics.robotDescription.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.ExternalForcePointDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.KinematicPointDescription;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LoopClosureConstraintDescription;
import us.ihmc.robotics.robotDescription.LoopClosurePinConstraintDescription;
import us.ihmc.robotics.robotDescription.OneDoFJointDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SensorDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.LidarSensorDefinition;
import us.ihmc.scs2.definition.robot.LoopClosureDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;

public class RobotDescriptionConverter
{
   public static RobotDefinition toRobotDefinition(RobotDescription robotDescription)
   {
      RigidBodyDefinition rootBody = new RigidBodyDefinition(robotDescription.getName() + "RootBody");

      for (JointDescription rootJointToCopy : robotDescription.getRootJoints())
         createAndAddJointsRecursive(rootBody, rootJointToCopy);

      for (JointDescription rootJointToCopy : robotDescription.getRootJoints())
         createLoopClosureJointsRecursive(rootBody, rootJointToCopy);

      RobotDefinition robotDefinition = new RobotDefinition(robotDescription.getName());
      robotDefinition.setRootBodyDefinition(rootBody);
      return robotDefinition;
   }

   public static void createAndAddJointsRecursive(RigidBodyDefinition parentBody, JointDescription jointToCopy)
   {
      JointDefinition jointDefinition = toJointDefinition(jointToCopy);
      parentBody.addChildJoint(jointDefinition);

      RigidBodyDefinition rigidBodyDefinition = toRigidBodyDefinition(jointToCopy.getLink());
      jointDefinition.setSuccessor(rigidBodyDefinition);

      for (JointDescription childJointToCopy : jointToCopy.getChildrenJoints())
         createAndAddJointsRecursive(rigidBodyDefinition, childJointToCopy);
   }

   public static RigidBodyDefinition toRigidBodyDefinition(LinkDescription source)
   {
      RigidBodyDefinition output = new RigidBodyDefinition(source.getName());
      output.setMass(source.getMass());
      output.setMomentOfInertia(source.getMomentOfInertiaCopy());
      output.setCenterOfMassOffset(source.getCenterOfMassOffset());
      output.addVisualDefinitions(VisualsConversionTools.toVisualDefinitions(source.getLinkGraphics()));
      return output;
   }

   public static JointDefinition toJointDefinition(JointDescription source)
   {
      if (source instanceof FloatingJointDescription)
         return toSixDoFJointDefinition((FloatingJointDescription) source);
      if (source instanceof PinJointDescription)
         return toRevoluteJointDefinition((PinJointDescription) source);
      if (source instanceof SliderJointDescription)
         return toPrismaticJointDefinition((SliderJointDescription) source);
      return null;
   }

   public static SixDoFJointDefinition toSixDoFJointDefinition(FloatingJointDescription source)
   {
      SixDoFJointDefinition output = new SixDoFJointDefinition();
      copyJointProperties(source, output);
      return output;
   }

   public static RevoluteJointDefinition toRevoluteJointDefinition(PinJointDescription source)
   {
      RevoluteJointDefinition output = new RevoluteJointDefinition();
      copyOneDofJointProperties(source, output);
      return output;
   }

   public static PrismaticJointDefinition toPrismaticJointDefinition(SliderJointDescription source)
   {
      PrismaticJointDefinition output = new PrismaticJointDefinition();
      copyOneDofJointProperties(source, output);
      return output;
   }

   private static void copyJointProperties(JointDescription source, JointDefinition destination)
   {
      destination.setName(source.getName());
      destination.getTransformToParent().getTranslation().set(source.getOffsetFromParentJoint());
      source.getIMUSensors().forEach(imu -> destination.addSensorDefinition(toIMUSensorDefinition(imu)));
      source.getForceSensors().forEach(forceSensor -> destination.addSensorDefinition(toWrenchSensorDefinition(forceSensor)));
      source.getLidarSensors().forEach(lidar -> destination.addSensorDefinition(toLidarSensorDefinition(lidar)));
      source.getCameraSensors().forEach(camera -> destination.addSensorDefinition(toCameraSensorDefinition(camera)));

      source.getKinematicPoints().forEach(kp -> destination.addKinematicPointDefinition(toKinematicPointDefinition(kp)));
      source.getExternalForcePoints().forEach(efp -> destination.addExternalWrenchPointDefinition(toExternalWrenchPointDefinition(efp)));
      source.getGroundContactPoints().forEach(gcp -> destination.addGroundContactPointDefinition(toGroundContactPointDefinition(gcp)));
   }

   private static void copyOneDofJointProperties(OneDoFJointDescription source, OneDoFJointDefinition destination)
   {
      copyJointProperties(source, destination);
      destination.setAxis(source.getJointAxis());
      destination.setPositionLimits(source.getLowerLimit(), source.getUpperLimit());
      destination.setVelocityLimits(source.getVelocityLimit());
      destination.setEffortLimits(source.getEffortLimit());
      destination.setStiction(source.getStiction());
      destination.setDamping(source.getDamping());
      double[] limitStopParameters = source.getLimitStopParameters();
      destination.setGainsSoftLimitStop(limitStopParameters[2], limitStopParameters[3]);
   }

   public static SensorDefinition toSensorDefinition(SensorDescription source)
   {
      if (source instanceof IMUSensorDescription)
         return toIMUSensorDefinition((IMUSensorDescription) source);
      if (source instanceof ForceSensorDescription)
         return toWrenchSensorDefinition((ForceSensorDescription) source);
      // TODO Copy other sensors
      return null;
   }

   private static IMUSensorDefinition toIMUSensorDefinition(IMUSensorDescription source)
   {
      IMUSensorDefinition output = new IMUSensorDefinition();
      copySensorProperties(source, output);
      //TODO Copy noise parameters
      return output;
   }

   private static WrenchSensorDefinition toWrenchSensorDefinition(ForceSensorDescription source)
   {
      WrenchSensorDefinition output = new WrenchSensorDefinition();
      copySensorProperties(source, output);
      return output;
   }

   private static LidarSensorDefinition toLidarSensorDefinition(LidarSensorDescription source)
   {
      LidarSensorDefinition output = new LidarSensorDefinition();
      copySensorProperties(source, output);
      output.setSweepYawMin(source.getSweepYawMin());
      output.setSweepYawMax(source.getSweepYawMax());
      output.setHeightPitchMin(source.getHeightPitchMin());
      output.setHeightPitchMax(source.getHeightPitchMax());
      output.setMinRange(source.getMinRange());
      output.setMaxRange(source.getMaxRange());
      output.setPointsPerSweep(source.getPointsPerSweep());
      output.setScanHeight(source.getScanHeight());
      return output;
   }

   private static CameraSensorDefinition toCameraSensorDefinition(CameraSensorDescription source)
   {
      CameraSensorDefinition output = new CameraSensorDefinition();
      copySensorProperties(source, output);
      // TODO The transform in the description is x-forward and z-up while the camera should be z-forward and y-down
      output.getTransformToJoint().appendYawRotation(-Math.PI / 2.0);
      output.getTransformToJoint().appendRollRotation(-Math.PI / 2.0);
      output.setFieldOfView(source.getFieldOfView());
      output.setClipNear(source.getClipNear());
      output.setClipFar(100000.0);//source.getClipFar()); // TODO Allows to view the entire scene, not sure if that's what we want
      output.setImageWidth(source.getImageWidth());
      output.setImageHeight(source.getImageHeight());
      output.setUpdatePeriod(1000 / 25); // 25Hz // TODO Weird this is not present in the description. 
      return output;
   }

   private static void copySensorProperties(SensorDescription source, SensorDefinition destination)
   {
      destination.setName(source.getName());
      destination.getTransformToJoint().set(source.getTransformToJoint());
   }

   private static KinematicPointDefinition toKinematicPointDefinition(KinematicPointDescription source)
   {
      return new KinematicPointDefinition(source.getName(), source.getOffsetFromJoint());
   }

   private static ExternalWrenchPointDefinition toExternalWrenchPointDefinition(ExternalForcePointDescription source)
   {
      return new ExternalWrenchPointDefinition(source.getName(), source.getOffsetFromJoint());
   }

   private static GroundContactPointDefinition toGroundContactPointDefinition(GroundContactPointDescription source)
   {
      return new GroundContactPointDefinition(source.getName(), source.getOffsetFromJoint(), source.getGroupIdentifier());
   }

   private static void createLoopClosureJointsRecursive(RigidBodyDefinition rootBody, JointDescription jointDescription)
   {
      List<LoopClosureConstraintDescription> constraintDescriptions = jointDescription.getChildrenConstraintDescriptions();

      for (LoopClosureConstraintDescription constraintDescription : constraintDescriptions)
      {
         if (!(constraintDescription instanceof LoopClosurePinConstraintDescription))
            throw new UnsupportedOperationException("Only " + LoopClosurePinConstraintDescription.class.getSimpleName() + " constraints are supported.");

         RevoluteJointDefinition jointDefinition = new RevoluteJointDefinition(constraintDescription.getName());
         // TODO Being a little lazy on figuring out the joint axis.
         if (EuclidCoreTools.isZero(constraintDescription.getConstraintMomentSubSpace().getM00(), 1.0e-12))
            jointDefinition.setAxis(Axis3D.X);
         else if (EuclidCoreTools.isZero(constraintDescription.getConstraintMomentSubSpace().getM11(), 1.0e-12))
            jointDefinition.setAxis(Axis3D.Y);
         else if (EuclidCoreTools.isZero(constraintDescription.getConstraintMomentSubSpace().getM22(), 1.0e-12))
            jointDefinition.setAxis(Axis3D.Z);
         else
            throw new IllegalStateException("Unable to retrieve the joint axis.");

         jointDefinition.getTransformToParent().getTranslation().set(constraintDescription.getOffsetFromParentJoint());
         LoopClosureDefinition loopClosureDefinition = new LoopClosureDefinition();
         loopClosureDefinition.setOffsetFromSuccessorParent(constraintDescription.getOffsetFromLinkParentJoint());
         loopClosureDefinition.setKpSoftConstraint(new Vector3D(constraintDescription.getProportionalGains()));
         loopClosureDefinition.setKdSoftConstraint(new Vector3D(constraintDescription.getDerivativeGains()));
         jointDefinition.setLoopClosureDefinition(loopClosureDefinition);

         JointDefinition parentJoint = RobotDefinition.findJointDefinition(rootBody, constraintDescription.getParentJoint().getName());
         parentJoint.getSuccessor().addChildJoint(jointDefinition);
         RigidBodyDefinition successor = RobotDefinition.findRigidBodyDefinition(rootBody, constraintDescription.getLink().getName());
         jointDefinition.setLoopClosureSuccessor(successor);
      }

      for (JointDescription childJointDescription : jointDescription.getChildrenJoints())
      {
         createLoopClosureJointsRecursive(rootBody, childJointDescription);
      }
   }
}
