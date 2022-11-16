package us.ihmc.valkyrie;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.modelFileLoaders.RobotDefinitionLoader;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class ValkyrieModelComparisonTest
{
   private static final double TRANSFORM_POSITION_EPS = 1.0e-4;
   private static final double TRANSFORM_ROTATION_EPS = 1.0e-7;
   private static final double AXIS_EPS = 0;
   private static final double MOI_EPS = 1.0e-6;
   private static final double MASS_EPS = 1.0e-5;
   private static final double LIMIT_EPS = 1.0e-4;

   @Test
   public void testSDFModelAgainstURDF()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      ClassLoader classLoader = getClass().getClassLoader();
      List<String> resourceDirectories = Arrays.asList(robotModel.getResourceDirectories());
      String modelName = robotModel.getJointMap().getModelName();
      JointNameMap<?> jointMap = robotModel.getJointMap();
      RobotDefinition urdfRobotDefinition;
      try
      {
         urdfRobotDefinition = RobotDefinitionLoader.loadURDFModel(classLoader.getResourceAsStream("models/val_description/urdf/valkyrie_sim.urdf"),
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
      RobotDefinition sdfRobotDefinition = RobotDefinitionLoader.loadSDFModel(classLoader.getResourceAsStream("models/val_description/sdf/valkyrie_sim.sdf"),
                                                                              resourceDirectories,
                                                                              classLoader,
                                                                              modelName,
                                                                              null,
                                                                              jointMap,
                                                                              true);
      try
      {
         assertEquals(sdfRobotDefinition, urdfRobotDefinition);
      }
      catch (AssertionFailedError e)
      {
         compareAndReportJointDifferences(sdfRobotDefinition, urdfRobotDefinition);

         compareAndReportRigidBodyDifferences(sdfRobotDefinition, urdfRobotDefinition);

         throw e;
      }
   }

   public void compareAndReportRigidBodyDifferences(RobotDefinition sdfRobotDefinition, RobotDefinition urdfRobotDefinition)
   {
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
            continue;
         }

         if (sdfBody.getName().contains("Thumb") || sdfBody.getName().contains("Finger") || sdfBody.getName().contains("Pinky"))
            continue; // TODO Ignoring fingers

         if (!EuclidCoreTools.epsilonEquals(sdfBody.getMass(), urdfBody.getMass(), MASS_EPS))
         {
            System.err.printf("Body (%s) mass mismatch: SDF %f, URDF %f\n", sdfBody.getName(), sdfBody.getMass(), urdfBody.getMass());
         }

         if (!sdfBody.getMomentOfInertia().epsilonEquals(urdfBody.getMomentOfInertia(), MOI_EPS))
         {
            System.err.printf("Body (%s) moment of inertia mismatch: \n\tSDF  %s, \n\tURDF %s\n",
                              sdfBody.getName(),
                              sdfBody.getMomentOfInertia(),
                              urdfBody.getMomentOfInertia());
         }

         if (!sdfBody.getInertiaPose().getTranslation().epsilonEquals(urdfBody.getInertiaPose().getTranslation(), TRANSFORM_POSITION_EPS)
               || !sdfBody.getInertiaPose().getRotation().epsilonEquals(urdfBody.getInertiaPose().getRotation(), TRANSFORM_ROTATION_EPS))
         {
            System.err.printf("Body (%s) inertia psoe mismatch: \n\tSDF  %s\n\tURDF %s\n",
                              sdfBody.getName(),
                              sdfBody.getInertiaPose(),
                              urdfBody.getInertiaPose());
         }

         if (sdfBody.getParentJoint() == null)
         {
            if (urdfBody.getParentJoint() != null)
               System.err.println("SDF %s is root, URDF body is not root.");
         }
         else if (urdfBody.getParentJoint() == null)
         {
            if (sdfBody.getParentJoint() != null)
               System.err.println("SDF %s is not root, URDF body is root.");
         }
         else if (!sdfBody.getParentJoint().getName().equals(urdfBody.getParentJoint().getName()))
         {
            System.err.printf("Body (%s) parent joint mismatch: SDF %s, URDF %s\n",
                              sdfBody.getName(),
                              sdfBody.getParentJoint().getName(),
                              urdfBody.getParentJoint().getName());
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
                  }
               }
            }
         }
      }
   }

   public void compareAndReportJointDifferences(RobotDefinition sdfRobotDefinition, RobotDefinition urdfRobotDefinition)
   {
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

      assertEquals(allSDFJoints.size(), allURDFJoints.size());

      for (int i = 0; i < allSDFJoints.size(); i++)
      {
         JointDefinition sdfJoint = allSDFJoints.get(i);
         JointDefinition urdfJoint = allURDFJoints.get(i);

         if (!sdfJoint.getName().equals(urdfJoint.getName()))
         {
            System.err.printf("Joint name mismatch: SDF %s, URDF %s\n", sdfJoint.getName(), urdfJoint.getName());
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
         }

         if (!sdfJoint.getSuccessor().getName().equals(urdfJoint.getSuccessor().getName()))
         {
            System.err.printf("Joint (%s) successor mismatch: SDF %s, URDF %s\n",
                              sdfJoint.getName(),
                              sdfJoint.getSuccessor().getName(),
                              urdfJoint.getSuccessor().getName());
         }

         if (!sdfJoint.getPredecessor().getName().equals(urdfJoint.getPredecessor().getName()))
         {
            System.err.printf("Joint (%s) predecessor mismatch: SDF %s, URDF %s\n",
                              sdfJoint.getName(),
                              sdfJoint.getPredecessor().getName(),
                              urdfJoint.getPredecessor().getName());
         }

         if (sdfJoint.getClass() != urdfJoint.getClass())
         {
            System.err.printf("Joint (%s) type mismatch: SDF %s, URDF %s\n",
                              sdfJoint.getName(),
                              sdfJoint.getClass().getSimpleName(),
                              urdfJoint.getClass().getSimpleName());
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
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getPositionLowerLimit(), urdfOneDoFJoint.getPositionLowerLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) position lower limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getPositionLowerLimit(),
                                 urdfOneDoFJoint.getPositionLowerLimit());
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getPositionUpperLimit(), urdfOneDoFJoint.getPositionUpperLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) position upper limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getPositionUpperLimit(),
                                 urdfOneDoFJoint.getPositionUpperLimit());
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getVelocityLowerLimit(), urdfOneDoFJoint.getVelocityLowerLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) velocity lower limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getVelocityLowerLimit(),
                                 urdfOneDoFJoint.getVelocityLowerLimit());
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getVelocityUpperLimit(), urdfOneDoFJoint.getVelocityUpperLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) velocity upper limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getVelocityUpperLimit(),
                                 urdfOneDoFJoint.getVelocityUpperLimit());
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getEffortLowerLimit(), urdfOneDoFJoint.getEffortLowerLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) effort lower limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getEffortLowerLimit(),
                                 urdfOneDoFJoint.getEffortLowerLimit());
            }

            if (!EuclidCoreTools.epsilonEquals(sdfOneDoFJoint.getEffortUpperLimit(), urdfOneDoFJoint.getEffortUpperLimit(), LIMIT_EPS))
            {
               System.err.printf("1-DoF Joint (%s) effort upper limit mismatch: SDF %f, URDF %f\n",
                                 sdfJoint.getName(),
                                 sdfOneDoFJoint.getEffortUpperLimit(),
                                 urdfOneDoFJoint.getEffortUpperLimit());
            }

            //            if (sdfOneDoFJoint.getDamping() != urdfOneDoFJoint.getDamping())
            //            {
            //               System.err.printf("1-DoF Joint (%s) damping mismatch: SDF %f, URDF %f\n",
            //                                 sdfJoint.getName(),
            //                                 sdfOneDoFJoint.getDamping(),
            //                                 urdfOneDoFJoint.getDamping());
            //            }
            //
            //            if (sdfOneDoFJoint.getStiction() != urdfOneDoFJoint.getStiction())
            //            {
            //               System.err.printf("1-DoF Joint (%s) stiction mismatch: SDF %f, URDF %f\n",
            //                                 sdfJoint.getName(),
            //                                 sdfOneDoFJoint.getStiction(),
            //                                 urdfOneDoFJoint.getStiction());
            //            }
         }
      }
   }
}
