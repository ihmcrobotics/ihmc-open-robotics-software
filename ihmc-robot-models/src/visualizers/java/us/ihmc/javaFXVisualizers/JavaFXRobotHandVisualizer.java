package us.ihmc.javaFXVisualizers;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
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

   public JavaFXRobotHandVisualizer(FullHumanoidRobotModelFactory fullRobotModelFactory, RobotSide robotSide)
   {
      fullRobotModel = fullRobotModelFactory.createFullRobotModel();
      this.robotSide = robotSide;

      loadRobotModelAndGraphics(fullRobotModelFactory);
   }

   private void loadRobotModelAndGraphics(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      RobotDescription robotDescription = fullRobotModelFactory.getRobotDescription();

      RigidBodyBasics handBody = fullRobotModel.getHand(robotSide).getParentJoint().getPredecessor();
      if (handBody != null)
      {
         JointBasics parentJoint = handBody.getParentJoint();
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
   }

   public void updateTransform(Affine transform)
   {
      rootNode.getTransforms().add(transform);
   }

   private void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode)
   {
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
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
