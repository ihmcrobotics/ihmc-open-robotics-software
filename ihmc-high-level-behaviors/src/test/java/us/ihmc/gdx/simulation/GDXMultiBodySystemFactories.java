package us.ihmc.gdx.simulation;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemFactories.JointBuilder;
import us.ihmc.mecano.tools.MultiBodySystemFactories.RigidBodyBuilder;
import us.ihmc.mecano.yoVariables.tools.YoMultiBodySystemFactories;
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
      GDXRigidBody GDXRigidBody = new GDXRigidBody(rigidBody);
      List<VisualDefinition> visualDefinitions = rigidBodyDefinition.getVisualDefinitions();

      if (graphicLoader != null)
      {
         graphicLoader.execute(() -> loadRigidBodyGraphic(visualDefinitions, GDXRigidBody, resourceClassLoader));
      }
      else
      {
         loadRigidBodyGraphic(visualDefinitions, GDXRigidBody, resourceClassLoader);
      }

      return GDXRigidBody;
   }

   private static void loadRigidBodyGraphic(List<VisualDefinition> visualDefinitions, GDXRigidBody GDXRigidBody, ClassLoader resourceClassLoader)
   {
      ModelInstance graphicNode = GDXVisualTools.collectNodes(visualDefinitions, resourceClassLoader);
      ReferenceFrame graphicFrame = GDXRigidBody.isRootBody() ? GDXRigidBody.getBodyFixedFrame() : GDXRigidBody.getParentJoint().getFrameAfterJoint();

      if (graphicNode != null)
         GDXRigidBody.setGraphics(new FrameGDXNode(graphicFrame, graphicNode));
   }
}
