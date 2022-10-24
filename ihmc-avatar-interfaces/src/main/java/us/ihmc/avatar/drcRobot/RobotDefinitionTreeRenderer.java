package us.ihmc.avatar.drcRobot;

import guru.nidi.graphviz.attribute.Color;
import guru.nidi.graphviz.engine.Format;
import guru.nidi.graphviz.engine.Graphviz;
import guru.nidi.graphviz.model.MutableGraph;
import guru.nidi.graphviz.model.MutableNode;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.graphviz.ReferenceFrameTreeViewer;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static guru.nidi.graphviz.model.Factory.mutGraph;
import static guru.nidi.graphviz.model.Factory.mutNode;

public class RobotDefinitionTreeRenderer
{
   private final List<JointDefinitionLabelProvider> jointLabelProviders = new ArrayList<>();
   private final List<RigidBodyDefinitionLabelProvider> rigidBodyLabelProviders = new ArrayList<>();

   public interface JointDefinitionLabelProvider
   {
      String getLabel(JointDefinition jointDefinition);
   }

   public interface RigidBodyDefinitionLabelProvider
   {
      String getLabel(RigidBodyDefinition rigidBodyDefinition);
   }

   public RobotDefinitionTreeRenderer(RobotDefinition robotDefinition, String filenamePostfix)
   {
      jointLabelProviders.add(JointDefinition::getName);
      rigidBodyLabelProviders.add(RigidBodyDefinition::getName);
      jointLabelProviders.add(joint -> joint.getClass().getSimpleName());
      jointLabelProviders.add(joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
         {
            return "axis = " + EuclidCoreIOTools.getTuple3DString(((OneDoFJointDefinition) joint).getAxis());
         }
         else
         {
            return null;
         }
      });
      jointLabelProviders.add(joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition oneDofJoint = ((OneDoFJointDefinition) joint);
            return "joint limits lower = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getPositionLowerLimit())
                   + " upper = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getPositionUpperLimit());
         }
         else
         {
            return null;
         }
      });
      jointLabelProviders.add(joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition oneDofJoint = ((OneDoFJointDefinition) joint);
            return "velocity limits lower = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getVelocityLowerLimit())
                   + " upper = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getVelocityUpperLimit());
         }
         else
         {
            return null;
         }
      });
      jointLabelProviders.add(joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition oneDofJoint = ((OneDoFJointDefinition) joint);
            return "torque (effort) limits lower = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getEffortLowerLimit())
                   + " upper = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getEffortUpperLimit());
         }
         else
         {
            return null;
         }
      });
      jointLabelProviders.add(joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition oneDofJoint = ((OneDoFJointDefinition) joint);
            return "damping = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getDamping());
         }
         else
         {
            return null;
         }
      });
      jointLabelProviders.add(joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition oneDofJoint = ((OneDoFJointDefinition) joint);
            return "damping velocity soft limit = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getDampingVelocitySoftLimit());
         }
         else
         {
            return null;
         }
      });
      jointLabelProviders.add(joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition oneDofJoint = ((OneDoFJointDefinition) joint);
            return "stiction = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getStiction());
         }
         else
         {
            return null;
         }
      });
      jointLabelProviders.add(joint ->
      {
         if (joint instanceof OneDoFJointDefinition)
         {
            OneDoFJointDefinition oneDofJoint = ((OneDoFJointDefinition) joint);
            return "soft limit stop kp = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getKpSoftLimitStop())
                   + " kd = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, oneDofJoint.getKdSoftLimitStop());
         }
         else
         {
            return null;
         }
      });
      jointLabelProviders.add(joint ->
      {
         return "translation to parent = " + EuclidCoreIOTools.getTuple3DString(joint.getTransformToParent().getTranslation());
      });
      jointLabelProviders.add(joint ->
      {
         return "rotation to parent (deg) = " + EuclidCoreIOTools.getYawPitchRollString(Math.toDegrees(joint.getTransformToParent().getYaw()),
                                                                                        Math.toDegrees(joint.getTransformToParent().getPitch()),
                                                                                        Math.toDegrees(joint.getTransformToParent().getRoll()));
      });
      rigidBodyLabelProviders.add(rigidBody ->
      {
         return "mass = " + ReferenceFrameTreeViewer.getLabelOf(rigidBody.getMass());
      });
      rigidBodyLabelProviders.add(rigidBody ->
      {
         return "moment of inertia =\n" + getMatrixLabel(rigidBody.getMomentOfInertia());
      });
      rigidBodyLabelProviders.add(rigidBody ->
      {
         return "inertia translation (CoM) to parent = " + EuclidCoreIOTools.getTuple3DString(rigidBody.getInertiaPose().getTranslation());
      });
      rigidBodyLabelProviders.add(rigidBody ->
      {
         return "inertia rotation to parent (deg) = " + EuclidCoreIOTools.getYawPitchRollString(Math.toDegrees(rigidBody.getInertiaPose().getYaw()),
                                                                                                Math.toDegrees(rigidBody.getInertiaPose().getPitch()),
                                                                                                Math.toDegrees(rigidBody.getInertiaPose().getRoll()));
      });
      rigidBodyLabelProviders.add(rigidBody ->
      {
         if (!rigidBody.getVisualDefinitions().isEmpty())
         {
            String visualDefinitionLabel = "visuals:\n";
            for (VisualDefinition visualDefinition : rigidBody.getVisualDefinitions())
            {
               if (visualDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
               {
                  ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) visualDefinition.getGeometryDefinition();
                  visualDefinitionLabel += visualDefinition.getName() + ": " + modelFileGeometryDefinition.getFileName() + "\n";
               }
               else if (visualDefinition.getGeometryDefinition() instanceof Box3DDefinition)
               {
                  Box3DDefinition boxGeometryDefinition = (Box3DDefinition) visualDefinition.getGeometryDefinition();
                  visualDefinitionLabel += "box = " + EuclidCoreIOTools.getStringOf("(",
                                                                                    " )",
                                                                                    ", ",
                                                                                    EuclidCoreIOTools.DEFAULT_FORMAT,
                                                                                    boxGeometryDefinition.getSizeX(),
                                                                                    boxGeometryDefinition.getSizeY(),
                                                                                    boxGeometryDefinition.getSizeZ());
               }
               else if (visualDefinition.getGeometryDefinition() instanceof Sphere3DDefinition)
               {
                  Sphere3DDefinition sphereGeometryDefinition = (Sphere3DDefinition) visualDefinition.getGeometryDefinition();
                  visualDefinitionLabel +=
                        "sphere radius = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, sphereGeometryDefinition.getRadius()) + " resolution = "
                        + sphereGeometryDefinition.getResolution() + "\n";
               }
               else
               {
                  LogTools.warn("Implement label for {}", visualDefinition.getGeometryDefinition().getClass().getSimpleName());
               }
               if (visualDefinition.getMaterialDefinition() != null)
               {
                  MaterialDefinition materialDefinition = visualDefinition.getMaterialDefinition();
                  if (materialDefinition.getName() != null)
                     visualDefinitionLabel += materialDefinition.getName() + ": ";
                  if (materialDefinition.getDiffuseColor() != null)
                     visualDefinitionLabel += "diffuse = " + EuclidCoreIOTools.getStringOf("(",
                                                                                           " )",
                                                                                           ", ",
                                                                                           EuclidCoreIOTools.DEFAULT_FORMAT,
                                                                                           materialDefinition.getDiffuseColor().getRed(),
                                                                                           materialDefinition.getDiffuseColor().getGreen(),
                                                                                           materialDefinition.getDiffuseColor().getBlue(),
                                                                                           materialDefinition.getDiffuseColor().getAlpha()) + "\n";
                  if (materialDefinition.getAmbientColor() != null)
                     visualDefinitionLabel += "ambient = " + EuclidCoreIOTools.getStringOf("RGBA (",
                                                                                           " )",
                                                                                           ", ",
                                                                                           EuclidCoreIOTools.DEFAULT_FORMAT,
                                                                                           materialDefinition.getAmbientColor().getRed(),
                                                                                           materialDefinition.getAmbientColor().getGreen(),
                                                                                           materialDefinition.getAmbientColor().getBlue(),
                                                                                           materialDefinition.getAmbientColor().getAlpha()) + "\n";
               }
               visualDefinitionLabel +=
                     "   translation to parent = " + EuclidCoreIOTools.getTuple3DString(visualDefinition.getOriginPose().getTranslation()) + "\n";
               YawPitchRoll yawPitchRollToParent = new YawPitchRoll(visualDefinition.getOriginPose().getRotationView());
               visualDefinitionLabel +=
                     "   rotation to parent (deg) = " + EuclidCoreIOTools.getYawPitchRollString(Math.toDegrees(yawPitchRollToParent.getYaw()),
                                                                                                Math.toDegrees(yawPitchRollToParent.getPitch()),
                                                                                                Math.toDegrees(yawPitchRollToParent.getRoll()))
               + "\n";
            }
            return visualDefinitionLabel;
         }
         else
            return null;
      });
      rigidBodyLabelProviders.add(rigidBody ->
      {
         if (!rigidBody.getCollisionShapeDefinitions().isEmpty())
         {
            String collisionShapeDefinitionsLabel = "collision shapes:\n";
            for (CollisionShapeDefinition collisionShape : rigidBody.getCollisionShapeDefinitions())
            {
               if (collisionShape.getGeometryDefinition() instanceof ModelFileGeometryDefinition)
               {
                  ModelFileGeometryDefinition modelFileGeometryDefinition = (ModelFileGeometryDefinition) collisionShape.getGeometryDefinition();
                  collisionShapeDefinitionsLabel += collisionShape.getName() + ": " + modelFileGeometryDefinition.getFileName() + "\n";
               }
               else if (collisionShape.getGeometryDefinition() instanceof Box3DDefinition)
               {
                  Box3DDefinition boxGeometryDefinition = (Box3DDefinition) collisionShape.getGeometryDefinition();
                  collisionShapeDefinitionsLabel += "box = " + EuclidCoreIOTools.getStringOf("(",
                                                                                    " )",
                                                                                    ", ",
                                                                                    EuclidCoreIOTools.DEFAULT_FORMAT,
                                                                                    boxGeometryDefinition.getSizeX(),
                                                                                    boxGeometryDefinition.getSizeY(),
                                                                                    boxGeometryDefinition.getSizeZ());
               }
               else if (collisionShape.getGeometryDefinition() instanceof Sphere3DDefinition)
               {
                  Sphere3DDefinition sphereGeometryDefinition = (Sphere3DDefinition) collisionShape.getGeometryDefinition();
                  collisionShapeDefinitionsLabel +=
                        "sphere radius = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, sphereGeometryDefinition.getRadius()) + " resolution = "
                        + sphereGeometryDefinition.getResolution();
               }
               else if (collisionShape.getGeometryDefinition() instanceof Cylinder3DDefinition)
               {
                  Cylinder3DDefinition cylinderGeometryDefinition = (Cylinder3DDefinition) collisionShape.getGeometryDefinition();
                  collisionShapeDefinitionsLabel +=
                        "cylinder radius = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, cylinderGeometryDefinition.getRadius())
                        + " length = " + String.format(EuclidCoreIOTools.DEFAULT_FORMAT, cylinderGeometryDefinition.getLength())
                        + " resolution = " + cylinderGeometryDefinition.getResolution();
               }
               else
               {
                  LogTools.warn("Implement label for {}", collisionShape.getGeometryDefinition().getClass().getSimpleName());
               }
               collisionShapeDefinitionsLabel +=
                     "   translation to parent = " + EuclidCoreIOTools.getTuple3DString(collisionShape.getOriginPose().getTranslation()) + "\n";
               collisionShapeDefinitionsLabel +=
                     "   rotation to parent (deg) = " + EuclidCoreIOTools.getYawPitchRollString(Math.toDegrees(collisionShape.getOriginPose().getYaw()),
                                                                                                Math.toDegrees(collisionShape.getOriginPose().getPitch()),
                                                                                                Math.toDegrees(collisionShape.getOriginPose().getRoll()))
                     + "\n";
            }
            return collisionShapeDefinitionsLabel;
         }
         else
            return null;
      });

      MutableGraph graph = mutGraph("MultiBodySystemView").setDirected(true);
      MutableNode rootNode = createRigidBodyNode(robotDefinition.getRootBodyDefinition(), graph);
      addChildrenToGraph(robotDefinition.getRootBodyDefinition(), rootNode, graph);

      try
      {
         Graphviz.fromGraph(graph).render(Format.PNG).toFile(new File(robotDefinition.getName() + filenamePostfix + ".png"));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private MutableNode createRigidBodyNode(RigidBodyDefinition rigidBodyDefinition, MutableGraph graph)
   {
      String label = rigidBodyLabelProviders.get(0).getLabel(rigidBodyDefinition);

      for (int i = 1; i < rigidBodyLabelProviders.size(); i++)
      {
         String additionalLabel = rigidBodyLabelProviders.get(i).getLabel(rigidBodyDefinition);
         if (additionalLabel != null)
            label += "\n" + additionalLabel;
      }

      MutableNode rigidBodyNode = mutNode(label);
      rigidBodyNode.add(Color.DARKORCHID);
      graph.add(rigidBodyNode);

      return rigidBodyNode;
   }

   private void addChildrenToGraph(RigidBodyDefinition currentBodyDefinition, MutableNode currentNode, MutableGraph graph)
   {
      for (JointDefinition childJoint : currentBodyDefinition.getChildrenJoints())
      {
         RigidBodyDefinition childBody = childJoint.getSuccessor();
         MutableNode childJointNode, childBodyNode;

         childJointNode = createJointNode(childJoint, graph);
         childBodyNode = createRigidBodyNode(childBody, graph);
         graph.addLink(currentNode.addLink(childJointNode));
         graph.addLink(childJointNode.addLink(childBodyNode));
         addChildrenToGraph(childBody, childBodyNode, graph);
      }
   }

   private MutableNode createJointNode(JointDefinition joint, MutableGraph graph)
   {
      String label = jointLabelProviders.get(0).getLabel(joint);

      for (int i = 1; i < jointLabelProviders.size(); i++)
      {
         String additionalLabel = jointLabelProviders.get(i).getLabel(joint);
         if (additionalLabel != null)
            label += "\n" + additionalLabel;
      }

      MutableNode jointNode = mutNode(label);
      jointNode.add(Color.DARKGREEN);
      graph.add(jointNode);

      return jointNode;
   }

   private static String getMatrixLabel(Matrix3DReadOnly matrix)
   {
      String ret = EuclidCoreIOTools.getStringOf("/", " \\\\\n", ", ", matrix.getM00(), matrix.getM01(), matrix.getM02());
      ret += EuclidCoreIOTools.getStringOf("|", " |\n", ", ", matrix.getM10(), matrix.getM11(), matrix.getM12());
      ret += EuclidCoreIOTools.getStringOf("\\\\", " /", ", ", matrix.getM20(), matrix.getM21(), matrix.getM22());
      return ret;
   }
}
