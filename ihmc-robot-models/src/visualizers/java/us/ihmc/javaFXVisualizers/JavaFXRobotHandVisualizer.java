package us.ihmc.javaFXVisualizers;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.JavaFXTools;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.grahics.GraphicsIDRobot;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class JavaFXRobotHandVisualizer
{
   private RobotSide robotSide;
   private GraphicsRobot graphicsRobot;
   private JavaFXGraphics3DNode robotRootNode;
   private final FullHumanoidRobotModel fullRobotModel;

   private final Group rootNode = new Group();
   private FramePose3D handBodyPoseToControlFrame;

   public JavaFXRobotHandVisualizer(FullHumanoidRobotModelFactory fullRobotModelFactory, RobotSide robotSide)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      this.robotSide = robotSide;

      loadRobotModelAndGraphics(fullRobotModelFactory);
   }

   private void loadRobotModelAndGraphics(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      RigidBodyTransform handControlFrameToWorld = new RigidBodyTransform();
      handControlFrameToWorld.set(fullRobotModel.getHandControlFrame(robotSide).getTransformToWorldFrame());
      RigidBodyTransform rootNodeToWorld = new RigidBodyTransform();

      RobotDescription robotDescription = fullRobotModelFactory.getRobotDescription();

      RigidBodyBasics handBody = fullRobotModel.getHand(robotSide).getParentJoint().getPredecessor();
      if (handBody != null)
      {
         JointBasics parentJoint = handBody.getParentJoint();
         rootNodeToWorld.set(handBody.getBodyFixedFrame().getTransformToWorldFrame());

         LinkDescription linkDescription = robotDescription.getLinkDescription(parentJoint.getName());
         if (linkDescription.getLinkGraphics() == null)
         {
            linkDescription.setLinkGraphics(new LinkGraphicsDescription());
         }
         graphicsRobot = new GraphicsIDRobot(robotSide.getCamelCaseNameForStartOfExpression() + "HandGraphics", handBody, robotDescription, false);

         robotRootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
         robotRootNode.setMouseTransparent(true);
         addNodesRecursively(graphicsRobot.getRootNode(), robotRootNode);
         robotRootNode.update();

         rootNode.getChildren().add(robotRootNode);
      }

      handBodyPoseToControlFrame = new FramePose3D(ReferenceFrame.getWorldFrame(), rootNodeToWorld);
      handBodyPoseToControlFrame.changeFrame(fullRobotModel.getHandControlFrame(robotSide));
   }

   public void updateTransform(RigidBodyTransform transform)
   {
      FramePose3D transformFromXboxController = new FramePose3D(ReferenceFrame.getWorldFrame(), transform);
      RigidBodyTransform transformControlFrameToHandBodyPose = new RigidBodyTransform(handBodyPoseToControlFrame.getOrientation(), handBodyPoseToControlFrame.getPosition());
      transformFromXboxController.appendTransform(transformControlFrameToHandBodyPose);

      RigidBodyTransform transformToViz = new RigidBodyTransform(transformFromXboxController.getOrientation(), transformFromXboxController.getPosition());

      Affine affine = JavaFXTools.createAffineFromQuaternionAndTuple(new Quaternion(transformToViz.getRotation()),
                                                                     new Point3D(transformToViz.getTranslation()));

      rootNode.getTransforms().clear();
      rootNode.getTransforms().add(affine);
   }

   private void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode)
   {
      AppearanceDefinition appearance;
      if (robotSide == RobotSide.RIGHT)
         appearance = YoAppearance.YellowGreen();
      else
         appearance = YoAppearance.Crimson();
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode, appearance);
      parentNode.addChild(node);
      graphics3dNode.getChildrenNodes().forEach(child -> addNodesRecursively(child, node));
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
