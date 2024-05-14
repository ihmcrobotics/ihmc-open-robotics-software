package us.ihmc.rdx.simulation.scs2;

import java.util.List;
import java.util.concurrent.Executor;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
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
   public static RDXRigidBody toRDXMultiBodySystem(RigidBodyReadOnly originalRootBody, RobotDefinition robotDefinition)
   {
      return toRDXMultiBodySystem(originalRootBody, null, robotDefinition);
   }

   public static RDXRigidBody toRDXMultiBodySystem(RigidBodyReadOnly originalRootBody, RobotDefinition robotDefinition, double scaleFactor)
   {
      return (RDXRigidBody) MultiBodySystemFactories.cloneMultiBodySystem(originalRootBody,
                                                                          null,
                                                                          "",
                                                                          newRDXRigidBodyBuilder(MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER,
                                                                                                 robotDefinition,
                                                                                                 null,
                                                                                                 scaleFactor,
                                                                                                 false),
                                                                          MultiBodySystemFactories.DEFAULT_JOINT_BUILDER);
   }

   public static RDXRigidBody toRDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition)
   {
      return toRDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, MultiBodySystemFactories.DEFAULT_JOINT_BUILDER);
   }

   public static RDXRigidBody toRDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   Executor graphicLoader)
   {
      return toRDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, MultiBodySystemFactories.DEFAULT_JOINT_BUILDER, graphicLoader);
   }

   public static RDXRigidBody toYoRDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                     ReferenceFrame cloneStationaryFrame,
                                                     RobotDefinition robotDefinition,
                                                     YoRegistry registry)
   {
      return toRDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, YoMultiBodySystemFactories.newYoJointBuilder(registry), null);
   }

   public static RDXRigidBody toYoRDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                     ReferenceFrame cloneStationaryFrame,
                                                     RobotDefinition robotDefinition,
                                                     YoRegistry registry,
                                                     Executor graphicLoader)
   {
      return toRDXMultiBodySystem(originalRootBody,
                                  cloneStationaryFrame,
                                  robotDefinition,
                                  YoMultiBodySystemFactories.newYoJointBuilder(registry),
                                  graphicLoader);
   }

   public static RDXRigidBody toRDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   JointBuilder jointBuilder)
   {
      return toRDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, jointBuilder, null);
   }

   public static RDXRigidBody toRDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   JointBuilder jointBuilder,
                                                   Executor graphicLoader)
   {
      return (RDXRigidBody) MultiBodySystemFactories.cloneMultiBodySystem(originalRootBody,
                                                                          cloneStationaryFrame,
                                                                          "",
                                                                          newRDXRigidBodyBuilder(robotDefinition, graphicLoader),
                                                                          jointBuilder);
   }

   public static RigidBodyBuilder newRDXRigidBodyBuilder(RobotDefinition robotDefinition)
   {
      return newRDXRigidBodyBuilder(robotDefinition, null);
   }

   public static RigidBodyBuilder newRDXRigidBodyBuilder(RobotDefinition robotDefinition, Executor graphicLoader)
   {
      return newRDXRigidBodyBuilder(MultiBodySystemFactories.DEFAULT_RIGID_BODY_BUILDER, robotDefinition, graphicLoader, RDXVisualTools.NO_SCALING, false);
   }

   public static RigidBodyBuilder newRDXRigidBodyBuilder(RigidBodyBuilder rigidBodyBuilder, RobotDefinition robotDefinition)
   {
      return newRDXRigidBodyBuilder(rigidBodyBuilder, robotDefinition, null, RDXVisualTools.NO_SCALING, false);
   }

   public static RigidBodyBuilder newRDXRigidBodyBuilder(RigidBodyBuilder rigidBodyBuilder,
                                                         RobotDefinition robotDefinition,
                                                         Executor graphicLoader,
                                                         double scaleFactor,
                                                         boolean createReferenceFrameGraphics)
   {
      return new RigidBodyBuilder()
      {
         @Override
         public RDXRigidBody buildRoot(String bodyName, RigidBodyTransformReadOnly transformToParent, ReferenceFrame parentStationaryFrame)
         {
            RigidBodyBasics rootBody = rigidBodyBuilder.buildRoot(bodyName, transformToParent, parentStationaryFrame);
            return toRDXRigidBody(rootBody,
                                  robotDefinition.getRigidBodyDefinition(rootBody.getName()),
                                  graphicLoader,
                                  scaleFactor,
                                  createReferenceFrameGraphics);
         }

         @Override
         public RDXRigidBody build(String bodyName,
                                   JointBasics parentJoint,
                                   Matrix3DReadOnly momentOfInertia,
                                   double mass,
                                   RigidBodyTransformReadOnly inertiaPose)
         {
            RigidBodyBasics rigidBody = rigidBodyBuilder.build(bodyName, parentJoint, momentOfInertia, mass, inertiaPose);
            return toRDXRigidBody(rigidBody,
                                  robotDefinition.getRigidBodyDefinition(rigidBody.getName()),
                                  graphicLoader,
                                  scaleFactor,
                                  createReferenceFrameGraphics);
         }
      };
   }

   public static RDXRigidBody toRDXRigidBody(RigidBodyBasics rigidBody,
                                             RigidBodyDefinition rigidBodyDefinition,
                                             Executor graphicLoader,
                                             double scaleFactor,
                                             boolean createReferenceFrameGraphics)
   {
      RDXRigidBody RDXRigidBody = new RDXRigidBody(rigidBody);
      // TODO (CD): Fix visual definition returned for the hand
      List<VisualDefinition> visualDefinitions = rigidBodyDefinition.getVisualDefinitions();
      List<CollisionShapeDefinition> collisionShapeDefinitions = rigidBodyDefinition.getCollisionShapeDefinitions();

      if (graphicLoader != null)
      {
         graphicLoader.execute(() -> loadRigidBodyGraphic(visualDefinitions,
                                                          collisionShapeDefinitions,
                                                          RDXRigidBody,
                                                          scaleFactor,
                                                          createReferenceFrameGraphics));
      }
      else
      {
         loadRigidBodyGraphic(visualDefinitions, collisionShapeDefinitions, RDXRigidBody, scaleFactor, createReferenceFrameGraphics);
      }

      return RDXRigidBody;
   }

   private static void loadRigidBodyGraphic(List<VisualDefinition> visualDefinitions,
                                            List<CollisionShapeDefinition> collisionShapeDefinitions,
                                            RDXRigidBody rdxRigidBody,
                                            double scaleFactor,
                                            boolean createReferenceFrameGraphics)
   {
      ReferenceFrame graphicFrame = rdxRigidBody.isRootBody() ? rdxRigidBody.getBodyFixedFrame() : rdxRigidBody.getParentJoint().getFrameAfterJoint();
      List<RDXVisualModelInstance> visualModels = RDXVisualTools.collectNodes(visualDefinitions, scaleFactor);
      List<RDXVisualModelInstance> collisionModels = RDXVisualTools.collectCollisionNodes(collisionShapeDefinitions, scaleFactor);
      if (!visualModels.isEmpty() || !collisionModels.isEmpty())
      {
         RDXFrameGraphicsNode visualGraphicsNode = new RDXFrameGraphicsNode(graphicFrame, createReferenceFrameGraphics);
         int i = 0;
         for (RDXVisualModelInstance visualModel : visualModels)
         {
            visualGraphicsNode.addModelPart(visualModel, rdxRigidBody.getName() + "Visual" + i);
         }
         rdxRigidBody.setVisualGraphics(visualGraphicsNode);
         RDXFrameGraphicsNode collisionGraphicsNode = new RDXFrameGraphicsNode(graphicFrame, createReferenceFrameGraphics);
         i = 0;
         for (RDXVisualModelInstance collisionModel : collisionModels)
         {
            collisionGraphicsNode.addModelPart(collisionModel, rdxRigidBody.getName() + "Collision" + i);
         }
         rdxRigidBody.setCollisionGraphics(collisionGraphicsNode);
      }
   }
}
