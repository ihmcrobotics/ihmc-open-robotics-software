package us.ihmc.rdx.simulation.scs2;

import java.util.List;
import java.util.concurrent.Executor;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.ui.gizmo.RDXVisualModelInstance;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories.JointBuilder;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;
import us.ihmc.mecano.yoVariables.tools.YoMultiBodySystemFactories;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RDXMultiBodySystemFactories
{
   public static RDXRigidBody toGDXMultiBodySystem(RigidBodyReadOnly originalRootBody, ReferenceFrame cloneStationaryFrame, RobotDefinition robotDefinition)
   {
      return toGDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, MultiBodySystemFactories.DEFAULT_JOINT_BUILDER);
   }

   public static RDXRigidBody toGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   Executor graphicLoader)
   {
      return toGDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, MultiBodySystemFactories.DEFAULT_JOINT_BUILDER, graphicLoader);
   }

   public static RDXRigidBody toYoGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                     ReferenceFrame cloneStationaryFrame,
                                                     RobotDefinition robotDefinition,
                                                     YoRegistry registry)
   {
      return toGDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, YoMultiBodySystemFactories.newYoJointBuilder(registry), null);
   }

   public static RDXRigidBody toYoGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                     ReferenceFrame cloneStationaryFrame,
                                                     RobotDefinition robotDefinition,
                                                     YoRegistry registry,
                                                     Executor graphicLoader)
   {
      return toGDXMultiBodySystem(originalRootBody,
                                  cloneStationaryFrame,
                                  robotDefinition,
                                  YoMultiBodySystemFactories.newYoJointBuilder(registry),
                                  graphicLoader);
   }

   public static RDXRigidBody toGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   JointBuilder jointBuilder)
   {
      return toGDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, jointBuilder, null);
   }

   public static RDXRigidBody toGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   JointBuilder jointBuilder,
                                                   Executor graphicLoader)
   {
      return (RDXRigidBody) MultiBodySystemFactories.cloneMultiBodySystem(originalRootBody,
                                                                          cloneStationaryFrame,
                                                                          "",
                                                                          newGDXRigidBodyBuilder(robotDefinition, graphicLoader),
                                                                          jointBuilder);
   }

   public static RigidBodyBuilder newGDXRigidBodyBuilder(RobotDefinition robotDefinition)
   {
      return newGDXRigidBodyBuilder(robotDefinition, null);
   }

   public static RigidBodyBuilder newGDXRigidBodyBuilder(RobotDefinition robotDefinition, Executor graphicLoader)
   {
      return newGDXRigidBodyBuilder(MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER,
                                    robotDefinition,
                                    graphicLoader,
                                    robotDefinition.getResourceClassLoader());
   }

   public static RigidBodyBuilder newGDXRigidBodyBuilder(RigidBodyBuilder rigidBodyBuilder, RobotDefinition robotDefinition)
   {
      return newGDXRigidBodyBuilder(rigidBodyBuilder, robotDefinition, null, robotDefinition.getResourceClassLoader());
   }

   public static RigidBodyBuilder newGDXRigidBodyBuilder(RigidBodyBuilder rigidBodyBuilder,
                                                         RobotDefinition robotDefinition,
                                                         Executor graphicLoader,
                                                         ClassLoader resourceClassLoader)
   {
      return new RigidBodyBuilder()
      {
         @Override
         public RDXRigidBody buildRoot(String bodyName, RigidBodyTransformReadOnly transformToParent, ReferenceFrame parentStationaryFrame)
         {
            RigidBodyBasics rootBody = rigidBodyBuilder.buildRoot(bodyName, transformToParent, parentStationaryFrame);
            return toGDXRigidBody(rootBody, robotDefinition.getRigidBodyDefinition(rootBody.getName()), graphicLoader, resourceClassLoader);
         }

         @Override
         public RDXRigidBody build(String bodyName, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransformReadOnly inertiaPose)
         {
            RigidBodyBasics rigidBody = rigidBodyBuilder.build(bodyName, parentJoint, momentOfInertia, mass, inertiaPose);
            return toGDXRigidBody(rigidBody, robotDefinition.getRigidBodyDefinition(rigidBody.getName()), graphicLoader, resourceClassLoader);
         }
      };
   }

   public static RDXRigidBody toGDXRigidBody(RigidBodyBasics rigidBody, RigidBodyDefinition rigidBodyDefinition, ClassLoader resourceClassLoader)
   {
      return toGDXRigidBody(rigidBody, rigidBodyDefinition, null, resourceClassLoader);
   }

   public static RDXRigidBody toGDXRigidBody(RigidBodyBasics rigidBody,
                                             RigidBodyDefinition rigidBodyDefinition,
                                             Executor graphicLoader,
                                             ClassLoader resourceClassLoader,
                                             Vector3D scale)
   {
      RDXRigidBody RDXRigidBody = new RDXRigidBody(rigidBody);
      List<VisualDefinition> visualDefinitions = rigidBodyDefinition.getVisualDefinitions();
      List<CollisionShapeDefinition> collisionShapeDefinitions = rigidBodyDefinition.getCollisionShapeDefinitions();

      if (graphicLoader != null)
      {
         graphicLoader.execute(() -> loadRigidBodyGraphic(visualDefinitions, collisionShapeDefinitions, RDXRigidBody, resourceClassLoader, scale));
      }
      else
      {
         loadRigidBodyGraphic(visualDefinitions, collisionShapeDefinitions, RDXRigidBody, resourceClassLoader, scale);
      }

      return RDXRigidBody;
   }

   public static RDXRigidBody toGDXRigidBody(RigidBodyBasics rigidBody,
                                             RigidBodyDefinition rigidBodyDefinition,
                                             Executor graphicLoader,
                                             ClassLoader resourceClassLoader)
   {
      RDXRigidBody RDXRigidBody = new RDXRigidBody(rigidBody);
      List<VisualDefinition> visualDefinitions = rigidBodyDefinition.getVisualDefinitions();
      List<CollisionShapeDefinition> collisionShapeDefinitions = rigidBodyDefinition.getCollisionShapeDefinitions();

      if (graphicLoader != null)
      {
         graphicLoader.execute(() -> loadRigidBodyGraphic(visualDefinitions, collisionShapeDefinitions, RDXRigidBody, resourceClassLoader));
      }
      else
      {
         loadRigidBodyGraphic(visualDefinitions, collisionShapeDefinitions, RDXRigidBody, resourceClassLoader);
      }

      return RDXRigidBody;
   }

   private static void loadRigidBodyGraphic(List<VisualDefinition> visualDefinitions,
                                            List<CollisionShapeDefinition> collisionShapeDefinitions,
                                            RDXRigidBody RDXRigidBody,
                                            ClassLoader resourceClassLoader,
                                            Vector3D scale)
   {
      ReferenceFrame graphicFrame = RDXRigidBody.isRootBody() ? RDXRigidBody.getBodyFixedFrame() : RDXRigidBody.getParentJoint().getFrameAfterJoint();
      List<RDXVisualModelInstance> visualModels = RDXVisualTools.collectNodes(visualDefinitions, resourceClassLoader);
      List<RDXVisualModelInstance> collisionModels = RDXVisualTools.collectCollisionNodes(collisionShapeDefinitions);
      if (!visualModels.isEmpty() || !collisionModels.isEmpty())
      {
         RDXFrameGraphicsNode visualGraphicsNode = new RDXFrameGraphicsNode(graphicFrame);
         int i = 0;
         for (RDXVisualModelInstance visualModel : visualModels)
         {
            visualGraphicsNode.addModelPart(visualModel, RDXRigidBody.getName() + "Visual" + i, scale);
         }
         RDXRigidBody.setVisualGraphics(visualGraphicsNode);
         RDXFrameGraphicsNode collisionGraphicsNode = new RDXFrameGraphicsNode(graphicFrame);
         i = 0;
         for (RDXVisualModelInstance collisionModel : collisionModels)
         {
            collisionGraphicsNode.addModelPart(collisionModel, RDXRigidBody.getName() + "Collision" + i, scale);
         }
         RDXRigidBody.setCollisionGraphics(collisionGraphicsNode);
      }
   }

   private static void loadRigidBodyGraphic(List<VisualDefinition> visualDefinitions,
                                            List<CollisionShapeDefinition> collisionShapeDefinitions,
                                            RDXRigidBody RDXRigidBody,
                                            ClassLoader resourceClassLoader)
   {
      ReferenceFrame graphicFrame = RDXRigidBody.isRootBody() ? RDXRigidBody.getBodyFixedFrame() : RDXRigidBody.getParentJoint().getFrameAfterJoint();
      List<RDXVisualModelInstance> visualModels = RDXVisualTools.collectNodes(visualDefinitions, resourceClassLoader);
      List<RDXVisualModelInstance> collisionModels = RDXVisualTools.collectCollisionNodes(collisionShapeDefinitions);
      if (!visualModels.isEmpty() || !collisionModels.isEmpty())
      {
         RDXFrameGraphicsNode visualGraphicsNode = new RDXFrameGraphicsNode(graphicFrame);
         int i = 0;
         for (RDXVisualModelInstance visualModel : visualModels)
         {
            visualGraphicsNode.addModelPart(visualModel, RDXRigidBody.getName() + "Visual" + i);
         }
         RDXRigidBody.setVisualGraphics(visualGraphicsNode);
         RDXFrameGraphicsNode collisionGraphicsNode = new RDXFrameGraphicsNode(graphicFrame);
         i = 0;
         for (RDXVisualModelInstance collisionModel : collisionModels)
         {
            collisionGraphicsNode.addModelPart(collisionModel, RDXRigidBody.getName() + "Collision" + i);
         }
         RDXRigidBody.setCollisionGraphics(collisionGraphicsNode);
      }
   }
}
