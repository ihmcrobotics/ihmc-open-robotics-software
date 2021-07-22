package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.ui.gizmo.DynamicGDXModel;
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

import java.util.List;
import java.util.concurrent.Executor;

public class GDXMultiBodySystemFactories
{
   public static GDXRigidBody toGDXMultiBodySystem(RigidBodyReadOnly originalRootBody, ReferenceFrame cloneStationaryFrame, RobotDefinition robotDefinition)
   {
      return toGDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, MultiBodySystemFactories.DEFAULT_JOINT_BUILDER);
   }

   public static GDXRigidBody toGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   Executor graphicLoader)
   {
      return toGDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, MultiBodySystemFactories.DEFAULT_JOINT_BUILDER, graphicLoader);
   }

   public static GDXRigidBody toYoGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                     ReferenceFrame cloneStationaryFrame,
                                                     RobotDefinition robotDefinition,
                                                     YoRegistry registry)
   {
      return toGDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, YoMultiBodySystemFactories.newYoJointBuilder(registry), null);
   }

   public static GDXRigidBody toYoGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
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

   public static GDXRigidBody toGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   JointBuilder jointBuilder)
   {
      return toGDXMultiBodySystem(originalRootBody, cloneStationaryFrame, robotDefinition, jointBuilder, null);
   }

   public static GDXRigidBody toGDXMultiBodySystem(RigidBodyReadOnly originalRootBody,
                                                   ReferenceFrame cloneStationaryFrame,
                                                   RobotDefinition robotDefinition,
                                                   JointBuilder jointBuilder,
                                                   Executor graphicLoader)
   {
      return (GDXRigidBody) MultiBodySystemFactories.cloneMultiBodySystem(originalRootBody,
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
         public GDXRigidBody buildRoot(String bodyName, RigidBodyTransform transformToParent, ReferenceFrame parentStationaryFrame)
         {
            RigidBodyBasics rootBody = rigidBodyBuilder.buildRoot(bodyName, transformToParent, parentStationaryFrame);
            return toGDXRigidBody(rootBody, robotDefinition.getRigidBodyDefinition(rootBody.getName()), graphicLoader, resourceClassLoader);
         }

         @Override
         public GDXRigidBody build(String bodyName, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransform inertiaPose)
         {
            RigidBodyBasics rigidBody = rigidBodyBuilder.build(bodyName, parentJoint, momentOfInertia, mass, inertiaPose);
            return toGDXRigidBody(rigidBody, robotDefinition.getRigidBodyDefinition(rigidBody.getName()), graphicLoader, resourceClassLoader);
         }
      };
   }

   public static GDXRigidBody toGDXRigidBody(RigidBodyBasics rigidBody, RigidBodyDefinition rigidBodyDefinition, ClassLoader resourceClassLoader)
   {
      return toGDXRigidBody(rigidBody, rigidBodyDefinition, null, resourceClassLoader);
   }

   public static GDXRigidBody toGDXRigidBody(RigidBodyBasics rigidBody,
                                             RigidBodyDefinition rigidBodyDefinition,
                                             Executor graphicLoader,
                                             ClassLoader resourceClassLoader)
   {
      GDXRigidBody gdxRigidBody = new GDXRigidBody(rigidBody);
      List<VisualDefinition> visualDefinitions = rigidBodyDefinition.getVisualDefinitions();
      List<CollisionShapeDefinition> collisionShapeDefinitions = rigidBodyDefinition.getCollisionShapeDefinitions();

      if (graphicLoader != null)
      {
         graphicLoader.execute(() -> loadRigidBodyGraphic(visualDefinitions, collisionShapeDefinitions, gdxRigidBody, resourceClassLoader));
      }
      else
      {
         loadRigidBodyGraphic(visualDefinitions, collisionShapeDefinitions, gdxRigidBody, resourceClassLoader);
      }

      return gdxRigidBody;
   }

   private static void loadRigidBodyGraphic(List<VisualDefinition> visualDefinitions,
                                            List<CollisionShapeDefinition> collisionShapeDefinitions,
                                            GDXRigidBody gdxRigidBody,
                                            ClassLoader resourceClassLoader)
   {
      DynamicGDXModel graphicNode = GDXVisualTools.collectNodes(visualDefinitions, resourceClassLoader);
      DynamicGDXModel collisionsNode = GDXVisualTools.collectCollisionNodes(collisionShapeDefinitions);
      ReferenceFrame graphicFrame = gdxRigidBody.isRootBody() ? gdxRigidBody.getBodyFixedFrame() : gdxRigidBody.getParentJoint().getFrameAfterJoint();

      if (graphicNode != null)
         gdxRigidBody.setGraphics(new FrameGDXNode(graphicFrame, graphicNode, collisionsNode));
   }
}
