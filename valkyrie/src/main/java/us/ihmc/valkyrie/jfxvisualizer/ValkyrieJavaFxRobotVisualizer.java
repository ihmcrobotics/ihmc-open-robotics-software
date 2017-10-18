package us.ihmc.valkyrie.jfxvisualizer;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.AmbientLight;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.CubeGraphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;

public class ValkyrieJavaFxRobotVisualizer extends Application
{
   private static final boolean VISUALIZE_LINK_LOCATIONS = true;
   private static final boolean DO_ANIMATION = false;
   private HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private GraphicsRobot graphicsRobot;
   private JavaFXGraphics3DNode rootNode;
   private double t;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("SDF Viewer");

      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false, ValkyrieConfigurationRoot.SIM_SDF_FILE);
      humanoidFloatingRootJointRobot = robotModel.createHumanoidFloatingRootJointRobot(false);

      ArrayList<Joint> joints = new ArrayList<>();
      joints.add(humanoidFloatingRootJointRobot.getRootJoint());

      if(VISUALIZE_LINK_LOCATIONS)
      {
         HashSet<Link> links = getAllLinks(joints, new HashSet<Link>());

         for (Link link : links)
         {
            AppearanceDefinition appearanceDefinition = YoAppearance.Yellow();
            appearanceDefinition.setTransparency(0.8);

            ArrayList<Graphics3DPrimitiveInstruction> instructions = new ArrayList<>();
            CubeGraphics3DInstruction e = new CubeGraphics3DInstruction(0.1, 0.1, 0.1, true);
            e.setAppearance(appearanceDefinition);
            instructions.add(e);

            Graphics3DObject oldLinkGraphics = link.getLinkGraphics();

            Graphics3DObject linkGraphics = new Graphics3DObject(instructions);

            ArrayList<Graphics3DPrimitiveInstruction> graphics3DInstructions = oldLinkGraphics.getGraphics3DInstructions();
            for (Graphics3DPrimitiveInstruction graphics3DInstruction : graphics3DInstructions)
            {
               linkGraphics.addInstruction(graphics3DInstruction);
            }

            link.setLinkGraphics(linkGraphics);
         }
      }

      graphicsRobot = new GraphicsRobot(humanoidFloatingRootJointRobot);

      graphicsRobot.update();

      rootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      addNodesRecursively(graphicsRobot.getRootNode(), rootNode);
      rootNode.update();

      View3DFactory view3DFactory = new View3DFactory(1024, 768);
      view3DFactory.addCameraController();

      view3DFactory.addNodeToView(new AmbientLight(Color.WHITE));
      view3DFactory.addPointLight(5, 0, 2, Color.GRAY);
      view3DFactory.addPointLight(-5, 0, 2, Color.GRAY);
      view3DFactory.addNodeToView(rootNode);

      primaryStage.setScene(view3DFactory.getScene());
      primaryStage.show();

      AnimationTimer animationTimer = new AnimationTimer()
      {

         @Override
         public void handle(long arg0)
         {
            updateRobotAndGraphics();
         }
      };
      animationTimer.start();

   }

   public void addNodesRecursively(Graphics3DNode graphics3dNode, JavaFXGraphics3DNode parentNode)
   {
      JavaFXGraphics3DNode node = new JavaFXGraphics3DNode(graphics3dNode);
      parentNode.addChild(node);

      for (Graphics3DNode child : graphics3dNode.getChildrenNodes())
      {
         addNodesRecursively(child, node);
      }
   }

   public void updateRobotAndGraphics()
   {
      if(DO_ANIMATION)
      {
         OneDegreeOfFreedomJoint[] oneDegreeOfFreedomJoints = humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoints();
         for (int i = 0; i < oneDegreeOfFreedomJoints.length; i++)
         {
            t += 0.001;
            double q = Math.sin(t);
            oneDegreeOfFreedomJoints[i].setQ(q);
         }
         humanoidFloatingRootJointRobot.update();
         graphicsRobot.update();
         rootNode.update();
      }
   }

   private static HashSet<Link> getAllLinks(ArrayList<Joint> joints, HashSet<Link> links)
   {
      for (Joint joint : joints)
      {
         links.add(joint.getLink());

         if (!joint.getChildrenJoints().isEmpty())
         {
            links.addAll(getAllLinks(joint.getChildrenJoints(), links));
         }
      }

      return links;
   }

   public static void main(String[] args) throws IOException
   {
      launch(args);
   }
}