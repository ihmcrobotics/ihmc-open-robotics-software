package us.ihmc.atlas.jfxvisualizer;

import java.io.IOException;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.scene.AmbientLight;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;

public class AtlasJavaFxRobotVisualizer extends Application
{
   private HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private GraphicsRobot graphicsRobot;
   private JavaFXGraphics3DNode rootNode;
   private double t;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("SDF Viewer");

      AtlasRobotModel robotModel = AtlasRobotModelFactory.createDefaultRobotModel();
      humanoidFloatingRootJointRobot = robotModel.createHumanoidFloatingRootJointRobot(false);

      graphicsRobot = new GraphicsRobot(humanoidFloatingRootJointRobot);
      graphicsRobot.update();

      rootNode = new JavaFXGraphics3DNode(graphicsRobot.getRootNode());
      addNodesRecursively(graphicsRobot.getRootNode(), rootNode);
      rootNode.update();

      View3DFactory view3DFactory = new View3DFactory(1024, 768);
      view3DFactory.addCameraController();
//      view3DFactory.setBackgroundColor(Color.WHITE);
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
      OneDegreeOfFreedomJoint[] oneDegreeOfFreedomJoints = humanoidFloatingRootJointRobot.getOneDegreeOfFreedomJoints();
//      for (int i = 0; i < oneDegreeOfFreedomJoints.length; i++)
//      {
//         t += 0.001;
//         double q = Math.sin(t);
//         oneDegreeOfFreedomJoints[i].setQ(q);
//      }
      humanoidFloatingRootJointRobot.update();
      graphicsRobot.update();
      rootNode.update();
   }

   public static void main(String[] args) throws IOException
   {
      launch(args);
   }
}