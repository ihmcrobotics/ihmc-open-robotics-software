package us.ihmc.zulu;

import us.ihmc.log.LogTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.zulu.parameters.model.ZuluPhysicalProperties;

import java.util.Arrays;
import java.util.Random;
import java.util.function.Consumer;

public class ZuluRigidBodyMutator implements Consumer<RobotDefinition>
{
   private ZuluPhysicalProperties physicalProperties;
   private final String[] imuSensorsToIgnore;

   public ZuluRigidBodyMutator(ZuluPhysicalProperties physicalProperties, String... imuSensorsToIgnore)
   {
      this.physicalProperties = physicalProperties;
      this.imuSensorsToIgnore = imuSensorsToIgnore;
   }

   @Override
   public void accept(RobotDefinition definition)
   {
      Random random = new Random(1);

      for (JointDefinition jointDefinition : definition.getAllJoints())
      {
         for (int sensorIdx = 0; sensorIdx < jointDefinition.getSensorDefinitions().size(); sensorIdx++)
         {
            SensorDefinition sensor = jointDefinition.getSensorDefinitions().get(sensorIdx);
            if (sensor instanceof IMUSensorDefinition imuSensor)
            {
               if (Arrays.stream(imuSensorsToIgnore).anyMatch(string -> string.equals(imuSensor.getName())))
               {
                  LogTools.info("Removing imu {} from the model, as we should be ignoring it", imuSensor.getName());
                  jointDefinition.getSensorDefinitions().remove(sensor);
                  break;
               }
            }
         }
      }

      for (RigidBodyDefinition rigidBody : definition.getAllRigidBodies())
      {
         if (rigidBody.getVisualDefinitions().size() == 0)
         {
            JointDefinition parentJoint = rigidBody.getParentJoint();
            if (parentJoint instanceof OneDoFJointDefinition)
            {
               OneDoFJointDefinition joint = (OneDoFJointDefinition) parentJoint;
               Cylinder3DDefinition cylinder = createCylinder();

               RigidBodyTransform cylinderTransform = new RigidBodyTransform();
               Vector3D axisRot = new Vector3D(joint.getAxis());
               axisRot.scale(Math.PI / 2.0);
               cylinderTransform.setRotationEulerAndZeroTranslation(axisRot.getY(), axisRot.getX(), 0.0);
               VisualDefinition visualDefinition = new VisualDefinition(cylinder,
                                                                        new ColorDefinition(random.nextInt(255), random.nextInt(255), random.nextInt(255)));
               visualDefinition.setOriginPose(cylinderTransform);
               rigidBody.addVisualDefinition(visualDefinition);
            }

            for (JointDefinition childJoint : rigidBody.getChildrenJoints())
            {
               if (childJoint instanceof OneDoFJointDefinition)
               {
                  OneDoFJointDefinition joint = (OneDoFJointDefinition) childJoint;
                  RigidBodyTransform transformToParent = new RigidBodyTransform(joint.getTransformToParent());

                  Box3DDefinition box = createBox(transformToParent.getTranslation());

                  VisualDefinition visualDefinition = new VisualDefinition(box,
                                                                           new ColorDefinition(random.nextInt(255), random.nextInt(255), random.nextInt(255)));

                  transformToParent.interpolate(new RigidBodyTransform(), 0.5);
                  visualDefinition.setOriginPose(transformToParent);
                  rigidBody.addVisualDefinition(visualDefinition);
               }
            }
         }
      }
   }

   private Box3DDefinition createFoot()
   {
      double footLength = physicalProperties.getActualFootLength();
      double footWidth = physicalProperties.getActualFootWidth();
      double footHeight = 0.02;
      Box3DDefinition box = new Box3DDefinition(footLength, footWidth, footHeight);
      return box;
   }

   private Box3DDefinition createBox(Vector3DBasics translationToParent)
   {
      double x = Math.max(0.05, Math.abs(translationToParent.getX()));
      double y = Math.max(0.05, Math.abs(translationToParent.getY()));
      double z = Math.max(0.05, Math.abs(translationToParent.getZ()));

      Box3DDefinition box = new Box3DDefinition(x, y, z);
      return box;
   }

   private Cylinder3DDefinition createCylinder()
   {
      double radius = 0.05;
      double length = 0.08;

      Cylinder3DDefinition cylinder = new Cylinder3DDefinition(length, radius);
      return cylinder;
   }
}
