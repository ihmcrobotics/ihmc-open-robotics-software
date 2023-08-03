package us.ihmc.valkyrie;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.LidarSensorDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SensorDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieModelComparisonTest
{
   private static final double TRANSFORM_POSITION_EPS = 1.0e-4;
   private static final double TRANSFORM_ROTATION_EPS = 1.0e-5;
   private static final double AXIS_EPS = 1.0e-4;
   private static final double MOI_EPS = 1.0e-6;
   private static final double MASS_EPS = 1.0e-5;
   private static final double LIMIT_EPS = 1.0e-4;

   @Test
   public void testDefaultModel()
   {
      testURDFAgainsSDF(ValkyrieRobotVersion.DEFAULT, "models/val_description/urdf/valkyrie_sim.urdf", "models/val_description/sdf/valkyrie_sim.sdf");
   }
   
   @Test
   public void testNoFingersModel()
   {
      testURDFAgainsSDF(ValkyrieRobotVersion.FINGERLESS, "models/val_description/urdf/valkyrie_sim_no_fingers.urdf", "models/val_description/sdf/valkyrie_sim_no_fingers.sdf");
   }
   
   @Test
   public void testArmMassSimModel()
   {
      testURDFAgainsSDF(ValkyrieRobotVersion.ARM_MASS_SIM, "models/val_description/urdf/valkyrie_sim_arm_mass_sim.urdf", "models/val_description/sdf/valkyrie_sim_arm_mass_sim.sdf");
   }
   
   @Test
   public void testNoArmsModel()
   {
      testURDFAgainsSDF(ValkyrieRobotVersion.ARMLESS, "models/val_description/urdf/valkyrie_sim_no_arms.urdf", "models/val_description/sdf/valkyrie_sim_no_arms.sdf");
   }

   private void testURDFAgainsSDF(ValkyrieRobotVersion version, String urdfPath, String sdfPath)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, version);
      ClassLoader classLoader = getClass().getClassLoader();
      List<String> resourceDirectories = Arrays.asList(robotModel.getResourceDirectories());
      String modelName = robotModel.getJointMap().getModelName();
      JointNameMap<?> jointMap = robotModel.getJointMap();
      RobotDefinition urdfRobotDefinition;
      try
      {
         urdfRobotDefinition = RobotDefinitionLoader.loadURDFModel(classLoader.getResourceAsStream(urdfPath),
                                                                   resourceDirectories,
                                                                   classLoader,
                                                                   modelName,
                                                                   null,
                                                                   jointMap,
                                                                   true);
      }
      catch (Exception e1)
      {
         e1.printStackTrace();
         throw e1;
      }
      RobotDefinition sdfRobotDefinition = RobotDefinitionLoader.loadSDFModel(classLoader.getResourceAsStream(sdfPath),
                                                                              resourceDirectories,
                                                                              classLoader,
                                                                              modelName,
                                                                              null,
                                                                              jointMap,
                                                                              true);
      boolean robotsEqual = compareAndReportJointDifferences(sdfRobotDefinition, urdfRobotDefinition);
      robotsEqual &= compareAndReportRigidBodyDifferences(sdfRobotDefinition, urdfRobotDefinition);
      assertTrue(robotsEqual);
   }

   public boolean compareAndReportRigidBodyDifferences(RobotDefinition sdfRobotDefinition, RobotDefinition urdfRobotDefinition)
   {
      boolean areRigidBodiesEqual = true;
      List<RigidBodyDefinition> allSDFBodies = sdfRobotDefinition.getAllRigidBodies();
      Collections.sort(allSDFBodies, (b1, b2) -> b1.getName().compareToIgnoreCase(b2.getName()));
      List<RigidBodyDefinition> allURDFBodies = urdfRobotDefinition.getAllRigidBodies();
      Collections.sort(allURDFBodies, (b1, b2) -> b1.getName().compareToIgnoreCase(b2.getName()));

      if (allSDFBodies.size() != allURDFBodies.size())
      {
         System.err.printf("Number of rigid-body mismatch:\nSDF %d bodies:\n[%s]\nURDF bodies %d:\n[%s]\n",
                           allSDFBodies.size(),
                           EuclidCoreIOTools.getCollectionString(", ", allSDFBodies, j -> j.getName()),
                           allURDFBodies.size(),
                           EuclidCoreIOTools.getCollectionString(", ", allURDFBodies, j -> j.getName()));
         areRigidBodiesEqual = false;
      }

      for (int i = allSDFBodies.size() - 1; i >= 0; i--)
      {
         String sdfJointName = allSDFBodies.get(i).getName();
         boolean hasURDFCounterpart = false;

         for (int j = 0; j < allURDFBodies.size(); j++)
         {
            if (allURDFBodies.get(j).getName().equals(sdfJointName))
            {
               hasURDFCounterpart = true;
               break;
            }
         }

         if (!hasURDFCounterpart)
         {
            allSDFBodies.remove(i);
         }
      }

      for (int i = allURDFBodies.size() - 1; i >= 0; i--)
      {
         String urdfJointName = allURDFBodies.get(i).getName();
         boolean hasSDFCounterpart = false;

         for (int j = 0; j < allSDFBodies.size(); j++)
         {
            if (allSDFBodies.get(j).getName().equals(urdfJointName))
            {
               hasSDFCounterpart = true;
               break;
            }
         }

         if (!hasSDFCounterpart)
         {
            allURDFBodies.remove(i);
         }
      }

      assertEquals(allSDFBodies.size(), allURDFBodies.size());

      for (int i = 0; i < allSDFBodies.size(); i++)
      {
         RigidBodyDefinition sdfBody = allSDFBodies.get(i);
         RigidBodyDefinition urdfBody = allURDFBodies.get(i);

         if (!sdfBody.getName().equals(urdfBody.getName()))
         {
            System.err.printf("Body name mismatch: SDF %s, URDF %s\n", sdfBody.getName(), urdfBody.getName());
            areRigidBodiesEqual = false;
            continue;
         }

         if (sdfBody.getName().contains("Thumb") || sdfBody.getName().contains("Finger") || sdfBody.getName().contains("Pinky"))
            continue; // TODO Ignoring fingers

         if (!EuclidCoreTools.epsilonEquals(sdfBody.getMass(), urdfBody.getMass(), MASS_EPS))
         {
            System.err.printf("Body (%s) mass mismatch: SDF %f, URDF %f\n", sdfBody.getName(), sdfBody.getMass(), urdfBody.getMass());
            areRigidBodiesEqual = false;
         }

         if (!sdfBody.getMomentOfInertia().epsilonEquals(urdfBody.getMomentOfInertia(), MOI_EPS))
         {
            System.err.printf("Body (%s) moment of inertia mismatch: \n\tSDF  %s, \n\tURDF %s\n",
                              sdfBody.getName(),
                              sdfBody.getMomentOfInertia(),
                              urdfBody.getMomentOfInertia());
            areRigidBodiesEqual = false;
         }

         if (!sdfBody.getInertiaPose().getTranslation().epsilonEquals(urdfBody.getInertiaPose().getTranslation(), TRANSFORM_POSITION_EPS)
               || !sdfBody.getInertiaPose().getRotation().epsilonEquals(urdfBody.getInertiaPose().getRotation(), TRANSFORM_ROTATION_EPS))
         {
            System.err.printf("Body (%s) inertia psoe mismatch: \n\tSDF  %s\n\tURDF %s\n",
                              sdfBody.getName(),
                              sdfBody.getInertiaPose(),
                              urdfBody.getInertiaPose());
            areRigidBodiesEqual = false;
         }

         if (sdfBody.getParentJoint() == null)
         {
            if (urdfBody.getParentJoint() != null)
            {
               System.err.println("SDF %s is root, URDF body is not root.");
               areRigidBodiesEqual = false;
            }
         }
         else if (urdfBody.getParentJoint() == null)
         {
            if (sdfBody.getParentJoint() != null)
            {
               System.err.println("SDF %s is not root, URDF body is root.");
               areRigidBodiesEqual = false;
            }
         }
         else if (!sdfBody.getParentJoint().getName().equals(urdfBody.getParentJoint().getName()))
         {
            System.err.printf("Body (%s) parent joint mismatch: SDF %s, URDF %s\n",
                              sdfBody.getName(),
                              sdfBody.getParentJoint().getName(),
                              urdfBody.getParentJoint().getName());
            areRigidBodiesEqual = false;
         }

         {
            Collections.sort(sdfBody.getChildrenJoints(), (j1, j2) -> j1.getName().compareTo(j2.getName()));
            Collections.sort(urdfBody.getChildrenJoints(), (j1, j2) -> j1.getName().compareTo(j2.getName()));

            if (sdfBody.getChildrenJoints().size() != urdfBody.getChildrenJoints().size())
            {
               System.err.printf("Body (%s) number of child joint mismatch:\nSDF %d children joints:\n[%s]\nURDF children joints %d:\n[%s]\n",
                                 sdfBody.getName(),
                                 sdfBody.getChildrenJoints().size(),
                                 EuclidCoreIOTools.getCollectionString(", ", sdfBody.getChildrenJoints(), j -> j.getName()),
                                 urdfBody.getChildrenJoints().size(),
                                 EuclidCoreIOTools.getCollectionString(", ", urdfBody.getChildrenJoints(), j -> j.getName()));

               areRigidBodiesEqual = false;
            }
            else
            {
               for (int k = 0; k < sdfBody.getChildrenJoints().size(); k++)
               {
                  if (!sdfBody.getChildrenJoints().get(k).getName().equals(urdfBody.getChildrenJoints().get(k).getName()))
                  {
                     System.err.printf("Body (%s) %dth child joint name mismatch: SDF child joint: %s, URDF child joint: %s",
                                       sdfBody.getName(),
                                       sdfBody.getChildrenJoints().get(k).getName(),
                                       urdfBody.getChildrenJoints().get(k).getName());
                     areRigidBodiesEqual = false;
                  }
               }
            }
         }
      }
      return areRigidBodiesEqual;
   }

   public boolean compareAndReportJointDifferences(RobotDefinition sdfRobotDefinition, RobotDefinition urdfRobotDefinition)
   {
      boolean areJointsEqual = true;
      List<JointDefinition> allSDFJoints = sdfRobotDefinition.getAllJoints();
      Collections.sort(allSDFJoints, (j1, j2) -> j1.getName().compareToIgnoreCase(j2.getName()));
      List<JointDefinition> allURDFJoints = urdfRobotDefinition.getAllJoints();
      Collections.sort(allURDFJoints, (j1, j2) -> j1.getName().compareToIgnoreCase(j2.getName()));

      if (allSDFJoints.size() != allURDFJoints.size())
      {
         System.err.printf("Number of joint mismatch:\nSDF %d joints:\n[%s]\nURDF joints %d:\n[%s]\n",
                           allSDFJoints.size(),
                           EuclidCoreIOTools.getCollectionString(", ", allSDFJoints, j -> j.getName()),
                           allURDFJoints.size(),
                           EuclidCoreIOTools.getCollectionString(", ", allURDFJoints, j -> j.getName()));
      }

      for (int i = allSDFJoints.size() - 1; i >= 0; i--)
      {
         String sdfJointName = allSDFJoints.get(i).getName();
         boolean hasURDFCounterpart = false;

         for (int j = 0; j < allURDFJoints.size(); j++)
         {
            if (allURDFJoints.get(j).getName().equals(sdfJointName))
            {
               hasURDFCounterpart = true;
               break;
            }
         }

         if (!hasURDFCounterpart)
         {
            allSDFJoints.remove(i);
         }
      }

      for (int i = allURDFJoints.size() - 1; i >= 0; i--)
      {
         String urdfJointName = allURDFJoints.get(i).getName();
         boolean hasSDFCounterpart = false;

         for (int j = 0; j < allSDFJoints.size(); j++)
         {
            if (allSDFJoints.get(j).getName().equals(urdfJointName))
            {
               hasSDFCounterpart = true;
               break;
            }
         }

         if (!hasSDFCounterpart)
         {
            allURDFJoints.remove(i);
         }
      }

      if (allSDFJoints.size() != allURDFJoints.size())
         return false;

      for (int i = 0; i < allSDFJoints.size(); i++)
      {
         JointDefinition sdfJoint = allSDFJoints.get(i);
         JointDefinition urdfJoint = allURDFJoints.get(i);

         if (!sdfJoint.getName().equals(urdfJoint.getName()))
         {
            System.err.printf("Joint name mismatch: SDF %s, URDF %s\n", sdfJoint.getName(), urdfJoint.getName());
            areJointsEqual = false;
            continue;
         }

         if (sdfJoint.getName().contains("Thumb") || sdfJoint.getName().contains("Finger") || sdfJoint.getName().contains("Pinky"))
            continue; // TODO Ignoring fingers

         if (!sdfJoint.getTransformToParent().getTranslation().epsilonEquals(urdfJoint.getTransformToParent().getTranslation(), TRANSFORM_POSITION_EPS)
               || !sdfJoint.getTransformToParent().getRotation().epsilonEquals(urdfJoint.getTransformToParent().getRotation(), TRANSFORM_ROTATION_EPS))
         {
            System.err.printf("Joint (%s) pose mismatch: \n\tSDF  %s\n\tURDF %s\n",
                              sdfJoint.getName(),
                              sdfJoint.getTransformToParent().toString(null),
                              urdfJoint.getTransformToParent().toString(null));
            areJointsEqual = false;
         }

         if (!sdfJoint.getSuccessor().getName().equals(urdfJoint.getSuccessor().getName()))
         {
            System.err.printf("Joint (%s) successor mismatch: SDF %s, URDF %s\n",
                              sdfJoint.getName(),
                              sdfJoint.getSuccessor().getName(),
                              urdfJoint.getSuccessor().getName());
            areJointsEqual = false;
         }

         if (!sdfJoint.getPredecessor().getName().equals(urdfJoint.getPredecessor().getName()))
         {
            System.err.printf("Joint (%s) predecessor mismatch: SDF %s, URDF %s\n",
                              sdfJoint.getName(),
                              sdfJoint.getPredecessor().getName(),
                              urdfJoint.getPredecessor().getName());
            areJointsEqual = false;
         }

         List<SensorDefinition> sdfSensors = sdfJoint.getSensorDefinitions();
         List<SensorDefinition> urdfSensors = urdfJoint.getSensorDefinitions();
         // This should be addressed with SCS2 0.11.2
         sdfSensors.forEach(sensor -> sensor.setName(sensor.getName().replace("___default__", "")));
         sdfSensors.forEach(sensor -> sensor.setName(sensor.getName().replace("__default__", "")));
         Collections.sort(sdfSensors, (s1, s2) -> s1.getName().compareTo(s2.getName()));
         Collections.sort(urdfSensors, (s1, s2) -> s1.getName().compareTo(s2.getName()));

         // Removing wrench sensors which are not handled by SDF
         urdfSensors.removeIf(sensor ->
         {
            if (!(sensor instanceof WrenchSensorDefinition))
               return false;
            System.out.printf("Removing URDF wrench sensor: %s, pose: %s\n", sensor.getName(), sensor.getTransformToJoint());
            return true;
         });

         if (sdfSensors.size() != urdfSensors.size())
         {
            System.err.printf("Number of sensor mismatch for joint %s:\nSDF %d sensors:\n[%s]\nURDF sensors %d:\n[%s]\n",
                              sdfJoint.getName(),
                              sdfSensors.size(),
                              EuclidCoreIOTools.getCollectionString(", ", sdfSensors, s -> s.getName()),
                              urdfSensors.size(),
                              EuclidCoreIOTools.getCollectionString(", ", urdfSensors, s -> s.getName()));
            areJointsEqual = false;
         }
         else
         {
            for (int sensorIndex = 0; sensorIndex < sdfSensors.size(); sensorIndex++)
            {
               SensorDefinition sdfSensor = sdfSensors.get(sensorIndex);
               SensorDefinition urdfSensor = urdfSensors.get(sensorIndex);

               if (!sdfSensor.getName().equals(urdfSensor.getName()))
               {
                  System.err.printf("Sensor name mismatch for joint %s: SDF %s, URDF %s\n", sdfJoint.getName(), sdfSensor.getName(), urdfSensor.getName());
                  areJointsEqual = false;
                  continue;
               }

               String sensorType = sdfSensor.getClass().getSimpleName().replace("SensorDefinition", " Sensor");

               if (!sdfSensor.getTransformToJoint().getTranslation().epsilonEquals(urdfSensor.getTransformToJoint().getTranslation(), TRANSFORM_POSITION_EPS)
                     || !sdfSensor.getTransformToJoint().getRotation().epsilonEquals(urdfSensor.getTransformToJoint().getRotation(), TRANSFORM_ROTATION_EPS))
               {
                  System.err.printf("%s (%s) pose mismatch for joint %s: \n\tSDF  %s\n\tURDF %s\n",
                                    sensorType,
                                    sdfSensor.getName(),
                                    sdfJoint.getName(),
                                    sdfSensor.getTransformToJoint().toString(null),
                                    urdfSensor.getTransformToJoint().toString(null));
                  areJointsEqual = false;
               }

               if (sdfSensor.getUpdatePeriod() != urdfSensor.getUpdatePeriod())
               {
                  System.err.printf("%s (%s) update period mismatch for joint %s: SDF  %d, URDF %d\n",
                                    sensorType,
                                    sdfSensor.getName(),
                                    sdfJoint.getName(),
                                    sdfSensor.getUpdatePeriod(),
                                    urdfSensor.getUpdatePeriod());
                  areJointsEqual = false;
               }

               if (sdfSensor.getClass() != urdfSensor.getClass())
               {
                  System.err.printf("%s (%s) type mismatch for joint %s: SDF %s, URDF %s\n",
                                    sdfSensor.getName(),
                                    sdfJoint.getName(),
                                    sdfSensor.getClass().getSimpleName(),
                                    urdfSensor.getClass().getSimpleName());
                  areJointsEqual = false;
               }
               else if (sdfSensor instanceof CameraSensorDefinition)
               {
                  CameraSensorDefinition sdfCameraSensor = (CameraSensorDefinition) sdfSensor;
                  CameraSensorDefinition urdfCameraSensor = (CameraSensorDefinition) urdfSensor;

                  if (sdfCameraSensor.getEnable() != urdfCameraSensor.getEnable())
                  {
                     System.err.printf("%s (%s) enable state mismatch for joint %s: SDF %s, URDF %s\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfCameraSensor.getEnable(),
                                       urdfCameraSensor.getEnable());
                     areJointsEqual = false;
                  }

                  if (sdfCameraSensor.getFieldOfView() != urdfCameraSensor.getFieldOfView())
                  {
                     System.err.printf("%s (%s) field of view mismatch for joint %s: SDF %s, URDF %s\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfCameraSensor.getFieldOfView(),
                                       urdfCameraSensor.getFieldOfView());
                     areJointsEqual = false;
                  }

                  if (sdfCameraSensor.getClipNear() != urdfCameraSensor.getClipNear())
                  {
                     System.err.printf("%s (%s) clip near mismatch for joint %s: SDF %s, URDF %s\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfCameraSensor.getClipNear(),
                                       urdfCameraSensor.getClipNear());
                     areJointsEqual = false;
                  }

                  if (sdfCameraSensor.getClipFar() != urdfCameraSensor.getClipFar())
                  {
                     System.err.printf("%s (%s) clip far mismatch for joint %s: SDF %s, URDF %s\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfCameraSensor.getClipFar(),
                                       urdfCameraSensor.getClipFar());
                     areJointsEqual = false;
                  }

                  if (sdfCameraSensor.getImageWidth() != urdfCameraSensor.getImageWidth())
                  {
                     System.err.printf("%s (%s) image width mismatch for joint %s: SDF %s, URDF %s\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfCameraSensor.getImageWidth(),
                                       urdfCameraSensor.getImageWidth());
                     areJointsEqual = false;
                  }

                  if (sdfCameraSensor.getImageHeight() != urdfCameraSensor.getImageHeight())
                  {
                     System.err.printf("%s (%s) image height mismatch for joint %s: SDF %s, URDF %s\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfCameraSensor.getImageHeight(),
                                       urdfCameraSensor.getImageHeight());
                     areJointsEqual = false;
                  }

                  if (!sdfCameraSensor.getDepthAxis().equals(urdfCameraSensor.getDepthAxis()))
                  {
                     System.err.printf("%s (%s) depth axis mismatch for joint %s: SDF %s, URDF %s\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfCameraSensor.getDepthAxis(),
                                       urdfCameraSensor.getDepthAxis());
                     areJointsEqual = false;
                  }

                  if (!sdfCameraSensor.getUpAxis().equals(urdfCameraSensor.getUpAxis()))
                  {
                     System.err.printf("%s (%s) up axis mismatch for joint %s: SDF %s, URDF %s\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfCameraSensor.getUpAxis(),
                                       urdfCameraSensor.getUpAxis());
                     areJointsEqual = false;
                  }
               }
               else if (sdfSensor instanceof IMUSensorDefinition)
               {
                  IMUSensorDefinition sdfIMUSensor = (IMUSensorDefinition) sdfSensor;
                  IMUSensorDefinition urdfIMUSensor = (IMUSensorDefinition) urdfSensor;

                  if (sdfIMUSensor.getAccelerationNoiseMean() != urdfIMUSensor.getAccelerationNoiseMean())
                  {
                     System.err.printf("%s (%s) up accel. noise mean mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfIMUSensor.getAccelerationNoiseMean(),
                                       urdfIMUSensor.getAccelerationNoiseMean());
                     areJointsEqual = false;
                  }

                  if (sdfIMUSensor.getAccelerationNoiseStandardDeviation() != urdfIMUSensor.getAccelerationNoiseStandardDeviation())
                  {
                     System.err.printf("%s (%s) up accel. noise st. dev. mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfIMUSensor.getAccelerationNoiseStandardDeviation(),
                                       urdfIMUSensor.getAccelerationNoiseStandardDeviation());
                     areJointsEqual = false;
                  }

                  if (sdfIMUSensor.getAccelerationBiasMean() != urdfIMUSensor.getAccelerationBiasMean())
                  {
                     System.err.printf("%s (%s) up accel. bias mean mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfIMUSensor.getAccelerationBiasMean(),
                                       urdfIMUSensor.getAccelerationBiasMean());
                     areJointsEqual = false;
                  }

                  if (sdfIMUSensor.getAccelerationBiasStandardDeviation() != urdfIMUSensor.getAccelerationBiasStandardDeviation())
                  {
                     System.err.printf("%s (%s) up accel. bias st. dev. mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfIMUSensor.getAccelerationBiasStandardDeviation(),
                                       urdfIMUSensor.getAccelerationBiasStandardDeviation());
                     areJointsEqual = false;
                  }

                  if (sdfIMUSensor.getAngularVelocityNoiseMean() != urdfIMUSensor.getAngularVelocityNoiseMean())
                  {
                     System.err.printf("%s (%s) up ang. vel. noise mean mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfIMUSensor.getAngularVelocityNoiseMean(),
                                       urdfIMUSensor.getAngularVelocityNoiseMean());
                     areJointsEqual = false;
                  }

                  if (sdfIMUSensor.getAngularVelocityNoiseStandardDeviation() != urdfIMUSensor.getAngularVelocityNoiseStandardDeviation())
                  {
                     System.err.printf("%s (%s) up ang. vel. noise st. dev. mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfIMUSensor.getAngularVelocityNoiseStandardDeviation(),
                                       urdfIMUSensor.getAngularVelocityNoiseStandardDeviation());
                     areJointsEqual = false;
                  }

                  if (sdfIMUSensor.getAngularVelocityBiasMean() != urdfIMUSensor.getAngularVelocityBiasMean())
                  {
                     System.err.printf("%s (%s) up ang. vel. bias mean mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfIMUSensor.getAngularVelocityBiasMean(),
                                       urdfIMUSensor.getAngularVelocityBiasMean());
                     areJointsEqual = false;
                  }

                  if (sdfIMUSensor.getAngularVelocityBiasStandardDeviation() != urdfIMUSensor.getAngularVelocityBiasStandardDeviation())
                  {
                     System.err.printf("%s (%s) up ang. vel. bias st. dev. mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfIMUSensor.getAngularVelocityBiasStandardDeviation(),
                                       urdfIMUSensor.getAngularVelocityBiasStandardDeviation());
                     areJointsEqual = false;
                  }
               }
               else if (sdfSensor instanceof LidarSensorDefinition)
               {
                  LidarSensorDefinition sdfLidarSensor = (LidarSensorDefinition) sdfSensor;
                  LidarSensorDefinition urdfLidarSensor = (LidarSensorDefinition) urdfSensor;

                  if (sdfLidarSensor.getSweepYawMin() != urdfLidarSensor.getSweepYawMin())
                  {
                     System.err.printf("%s (%s) sweep yaw min mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getSweepYawMin(),
                                       urdfLidarSensor.getSweepYawMin());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getSweepYawMax() != urdfLidarSensor.getSweepYawMax())
                  {
                     System.err.printf("%s (%s) sweep yaw max mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getSweepYawMax(),
                                       urdfLidarSensor.getSweepYawMax());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getHeightPitchMin() != urdfLidarSensor.getHeightPitchMin())
                  {
                     System.err.printf("%s (%s) height pitch min mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getHeightPitchMin(),
                                       urdfLidarSensor.getHeightPitchMin());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getHeightPitchMax() != urdfLidarSensor.getHeightPitchMax())
                  {
                     System.err.printf("%s (%s) height pitch max mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getHeightPitchMax(),
                                       urdfLidarSensor.getHeightPitchMax());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getMinRange() != urdfLidarSensor.getMinRange())
                  {
                     System.err.printf("%s (%s) min range mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getMinRange(),
                                       urdfLidarSensor.getMinRange());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getMaxRange() != urdfLidarSensor.getMaxRange())
                  {
                     System.err.printf("%s (%s) min range mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getMaxRange(),
                                       urdfLidarSensor.getMaxRange());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getRangeResolution() != urdfLidarSensor.getRangeResolution())
                  {
                     System.err.printf("%s (%s) range resolution mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getRangeResolution(),
                                       urdfLidarSensor.getRangeResolution());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getPointsPerSweep() != urdfLidarSensor.getPointsPerSweep())
                  {
                     System.err.printf("%s (%s) points per sweep mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getPointsPerSweep(),
                                       urdfLidarSensor.getPointsPerSweep());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getScanHeight() != urdfLidarSensor.getScanHeight())
                  {
                     System.err.printf("%s (%s) scan height mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getScanHeight(),
                                       urdfLidarSensor.getScanHeight());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getGaussianNoiseMean() != urdfLidarSensor.getGaussianNoiseMean())
                  {
                     System.err.printf("%s (%s) gaussian noise mean mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getGaussianNoiseMean(),
                                       urdfLidarSensor.getGaussianNoiseMean());
                     areJointsEqual = false;
                  }

                  if (sdfLidarSensor.getGaussianNoiseStandardDeviation() != urdfLidarSensor.getGaussianNoiseStandardDeviation())
                  {
                     System.err.printf("%s (%s) gaussian noise st. dev. mismatch for joint %s: SDF %f, URDF %f\n",
                                       sensorType,
                                       sdfSensor.getName(),
                                       sdfJoint.getName(),
                                       sdfLidarSensor.getGaussianNoiseStandardDeviation(),
                                       urdfLidarSensor.getGaussianNoiseStandardDeviation());
                     areJointsEqual = false;
                  }
               }
               else if (sdfSensor instanceof WrenchSensorDefinition)
               {
                  // Nothing more to test.
               }
            }
         }

         if (sdfJoint.getClass() != urdfJoint.getClass())
         {
            System.err.printf("Joint (%s) type mismatch: SDF %s, URDF %s\n",
                              sdfJoint.getName(),
                              sdfJoint.getClass().getSimpleName(),
                              urdfJoint.getClass().getSimpleName());
            areJointsEqual = false;
         }
         else if (sdfJoint instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition sdfOneDoFJoint = (OneDoFJointDefinition) sdfJoint;
            OneDoFJointDefinition urdfOneDoFJoint = (OneDoFJointDefinition) urdfJoint;

            if (!sdfOneDoFJoint.getAxis().epsilonEquals(urdfOneDoFJoint.getAxis(), AXIS_EPS))
            {
               System.err.printf("1-DoF Joint (%s) axis mismatch: SDF %s, URDF %s\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getAxis().toString(null),
                                 urdfOneDoFJoint.getAxis().toString(null));
               areJointsEqual = false;
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getPositionLowerLimit(), urdfOneDoFJoint.getPositionLowerLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) position lower limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getPositionLowerLimit(),
                                 urdfOneDoFJoint.getPositionLowerLimit());
               areJointsEqual = false;
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getPositionUpperLimit(), urdfOneDoFJoint.getPositionUpperLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) position upper limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getPositionUpperLimit(),
                                 urdfOneDoFJoint.getPositionUpperLimit());
               areJointsEqual = false;
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getVelocityLowerLimit(), urdfOneDoFJoint.getVelocityLowerLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) velocity lower limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getVelocityLowerLimit(),
                                 urdfOneDoFJoint.getVelocityLowerLimit());
               areJointsEqual = false;
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getVelocityUpperLimit(), urdfOneDoFJoint.getVelocityUpperLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) velocity upper limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getVelocityUpperLimit(),
                                 urdfOneDoFJoint.getVelocityUpperLimit());
               areJointsEqual = false;
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getEffortLowerLimit(), urdfOneDoFJoint.getEffortLowerLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) effort lower limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getEffortLowerLimit(),
                                 urdfOneDoFJoint.getEffortLowerLimit());
               areJointsEqual = false;
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getEffortUpperLimit(), urdfOneDoFJoint.getEffortUpperLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) effort upper limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getEffortUpperLimit(),
                                 urdfOneDoFJoint.getEffortUpperLimit());
               areJointsEqual = false;
            }

            //            if (sdfOneDoFJoint.getDamping() != urdfOneDoFJoint.getDamping())
            //            {
            //               System.err.printf("1-DoF Joint (%s) damping mismatch: SDF %f, URDF %f\n",
            //                                 sdfJoint.getName(),
            //                                 sdfOneDoFJoint.getDamping(),
            //                                 urdfOneDoFJoint.getDamping());
            //               areRobotDefinitionsEqual = false;
            //            }
            //
            //            if (sdfOneDoFJoint.getStiction() != urdfOneDoFJoint.getStiction())
            //            {
            //               System.err.printf("1-DoF Joint (%s) stiction mismatch: SDF %f, URDF %f\n",
            //                                 sdfJoint.getName(),
            //                                 sdfOneDoFJoint.getStiction(),
            //                                 urdfOneDoFJoint.getStiction());
            //               areRobotDefinitionsEqual = false;
            //            }
         }
      }
      return areJointsEqual;
   }
}
