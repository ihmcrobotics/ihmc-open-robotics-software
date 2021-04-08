package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.model.Node;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.live.GDXROS1PointCloudVisualizer;
import us.ihmc.gdx.ui.graphics.live.GDXROS1BoxVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.BoxSubscriber;
import us.ihmc.utilities.ros.subscriber.BoxesSubscriber;

import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.g3d.utils.shapebuilders.BoxShapeBuilder;

import us.ihmc.utilities.ros.publisher.RosTunningParamPublisher;
import lidar_obstacle_detection.TunningParam;


public class AtlasGDXOusterPointCloudViewer
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI();
   ImFloat tunableParameter1 = new ImFloat(0.1f);
   ImFloat tunableParameter2 = new ImFloat(0.1f);
   ImFloat tunableParameter3 = new ImFloat(0.1f);
   ImFloat tunableParameter4 = new ImFloat(0.1f);
   ImFloat tunableParameter5 = new ImFloat(0.1f);
   ImFloat tunableParameter6 = new ImFloat(0.1f);
   ImFloat tunableParameter7 = new ImFloat(0.1f);
   ImFloat tunableParameter8 = new ImFloat(0.1f);
   ImFloat tunableParameter9 = new ImFloat(0.1f);
   private final RosTunningParamPublisher rosTunningParamPublisher;


   //   private final BoxesSubscriber BoxesSubscriber = new BoxesSubscriber();
//   private BoxDemoModel2BoxDemoModel2BoxDemoModel2 boxVisualizer;



   public AtlasGDXOusterPointCloudViewer()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "ouster_point_cloud_viewer");
      rosTunningParamPublisher = new RosTunningParamPublisher();
      ros1Node.attachPublisher("/TunningParam", rosTunningParamPublisher);

      GDXROS1PointCloudVisualizer ros1PointCloudVisualizer = new GDXROS1PointCloudVisualizer(ros1Node,
                                                                                             "/downsampled_cloud_road",
                                                                                             ReferenceFrame.getWorldFrame(),
                                                                                             new RigidBodyTransform());

      GDXROS1PointCloudVisualizer ros1PointCloudVisualizer2 = new GDXROS1PointCloudVisualizer(ros1Node,
                                                                                             "/downsampled_cloud_obstacle",
                                                                                             ReferenceFrame.getWorldFrame(),
                                                                                             new RigidBodyTransform());

      BoxDemoModel2 boxVisualizer = new BoxDemoModel2(ros1Node, "/boxes",
                                        ReferenceFrame.getWorldFrame(), new RigidBodyTransform());


//      GDXROS1BoxVisualizer ros1BoxVisualizer = new GDXROS1BoxVisualizer(ros1Node,
//                                                                                              "/boxes",
//                                                                                              ReferenceFrame.getWorldFrame(),
//                                                                                              new RigidBodyTransform());
//      ModelInstance BoxDemoModel2 = ;
//      ModelInstance BoxDemoModel2 = new BoxDemoModel2(ros1Node, "/boxes").newInstance();

      //      ros1Node.attachSubscriber("/boxes", BoxesSubscriber);

      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         public void create()
         {
            baseUI.create();
            baseUI.getSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));


            ros1PointCloudVisualizer.create();
            ros1PointCloudVisualizer.setEnabled(true);
            ros1PointCloudVisualizer2.create();
            ros1PointCloudVisualizer2.setEnabled(true);
            boxVisualizer.create();
            boxVisualizer.setEnabled(true);

            //            ros1BoxVisualizer.create();
//            ros1BoxVisualizer.setEnabled(true);

            baseUI.getSceneManager().addRenderableProvider(ros1PointCloudVisualizer);
            baseUI.getSceneManager().addRenderableProvider(ros1PointCloudVisualizer2);
//            baseUI.getSceneManager().addRenderableProvider(ros1BoxVisualizer);

            baseUI.getSceneManager().addRenderableProvider(boxVisualizer);

//            baseUI.getSceneManager().addRenderableProvider(ros1BoxVisualizer);

            ros1Node.execute();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            boolean tunningParamsChanged = false;
            ImGui.begin("Stats");
            ImGui.text("/downsampled_cloud_road");

            //            receivedPlot.render(receivedCount);
            tunningParamsChanged |= ImGui.sliderFloat("TuneX", tunableParameter1.getData(), 0.0f, 1.0f);
            tunningParamsChanged |= ImGui.sliderFloat("TuneY", tunableParameter2.getData(), 0.0f, 1.0f);
            tunningParamsChanged |= ImGui.sliderFloat("TuneZ", tunableParameter3.getData(), 0.0f, 1.0f);
//            ImGui.dragFloat("TuneYaw", tuneYaw.getData(), 0.0001f, 0.01f, 0.08f);
//            ImGui.dragFloat("TunePitch", tunePitch.getData(), 0.0001f, 20.0f, 30.0f);
//            ImGui.dragFloat("TuneRoll", tuneRoll.getData(), 0.0001f, -5.0f, 5.0f);
            ImGui.text("/downsampled_cloud_obstacle");
            //            receivedPlot.render(receivedCount);
            tunningParamsChanged |= ImGui.sliderFloat("Tune1", tunableParameter4.getData(), 0.0f, 1.0f);
            tunningParamsChanged |= ImGui.sliderFloat("Tune2", tunableParameter5.getData(), 0.0f, 1.0f);
            tunningParamsChanged |= ImGui.sliderFloat("Tune3", tunableParameter6.getData(), 0.0f, 1.0f);

            ImGui.text("/box");
            //            receivedPlot.render(receivedCount);
            tunningParamsChanged |= ImGui.sliderFloat("Tune4", tunableParameter7.getData(), 0.0f, 1.0f);
            tunningParamsChanged |= ImGui.sliderFloat("Tune5", tunableParameter8.getData(), 0.0f, 1.0f);
            tunningParamsChanged |= ImGui.sliderFloat("Tune6", tunableParameter9.getData(), 0.0f, 1.0f);

            ImGui.end();
//            if (tunningParamsChanged)
//            {
            System.out.println("(double) tunableParameter1.getData()[0]"+(double) tunableParameter1.getData()[0]);
            System.out.println("(double) tunableParameter2.getData()[0]"+(double) tunableParameter2.getData()[0]);
            System.out.println("(double) tunableParameter3.getData()[0]"+(double) tunableParameter3.getData()[0]);
            System.out.println("(double) tunableParameter4.getData()[0]"+(double) tunableParameter4.getData()[0]);
            System.out.println("(double) tunableParameter5.getData()[0]"+(double) tunableParameter5.getData()[0]);
            System.out.println("(double) tunableParameter6.getData()[0]"+(double) tunableParameter6.getData()[0]);
            System.out.println("(double) tunableParameter7.getData()[0]"+(double) tunableParameter7.getData()[0]);
            System.out.println("(double) tunableParameter8.getData()[0]"+(double) tunableParameter8.getData()[0]);
            System.out.println("(double) tunableParameter9.getData()[0]"+(double) tunableParameter9.getData()[0]);

//            rosTunningParamPublisher.publish(0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1);

//            rosTunningParamPublisher.publish((double) tunableParameter1.getData()[0], (double) tunableParameter2.getData()[0], (double) tunableParameter3.getData()[0],
//                                                (double) tunableParameter4.getData()[0], (double) tunableParameter5.getData()[0], (double) tunableParameter6.getData()[6],
//                                                (double) tunableParameter7.getData()[0], (double) tunableParameter8.getData()[0], (double) tunableParameter9.getData()[0]);

//            }
//            System.out.println("tunableParameter1.getData()[0]:"+tunableParameter1.getData()[0]);
//            rosTunningParamPublisher.publish((double) tunableParameter1.getData()[0], (double) tunableParameter2.getData()[0], (double) tunableParameter3.getData()[0],
//                                             (double) tunableParameter4.getData()[0], (double) tunableParameter5.getData()[0], (double) tunableParameter6.getData()[6],
//                                             (double) tunableParameter7.getData()[0], (double) tunableParameter8.getData()[0], (double) tunableParameter9.getData()[0]);

            ros1PointCloudVisualizer.updateMeshcolor(0.0f);
            ros1PointCloudVisualizer2.updateMeshcolor(1.0f);
            boxVisualizer.update();

            //            ros1BoxVisualizer.updateMesh();

            baseUI.renderEnd();

         }
         @Override
         public void dispose()
         {
            baseUI.dispose();
         }

      }, getClass().getSimpleName(), 1100, 800);
   }


   public static void main(String[] args)
   {
      new AtlasGDXOusterPointCloudViewer();
   }
}

