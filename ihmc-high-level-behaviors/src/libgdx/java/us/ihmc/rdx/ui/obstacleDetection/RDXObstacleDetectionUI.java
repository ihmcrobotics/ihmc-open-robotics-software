package us.ihmc.rdx.ui.obstacleDetection;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.live.ros1.RDXROS1BoxVisualizer;
import us.ihmc.rdx.ui.graphics.live.ros1.RDXROS1PointCloudVisualizer;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

import us.ihmc.utilities.ros.publisher.RosTunningParamPublisher;

public class RDXObstacleDetectionUI
{
   public static final String APPLICATION_NAME = "Object Detection UI";
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/libgdx/resources",
                                                  APPLICATION_NAME);
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

   public RDXObstacleDetectionUI()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "obstacle_detection_ui");
      rosTunningParamPublisher = new RosTunningParamPublisher();
      ros1Node.attachPublisher("/TunningParam", rosTunningParamPublisher);

      RDXROS1PointCloudVisualizer ros1PointCloudVisualizer = new RDXROS1PointCloudVisualizer("Road point cloud", "/downsampled_cloud_road");
      ros1PointCloudVisualizer.subscribe(ros1Node);

      RDXROS1PointCloudVisualizer ros1PointCloudVisualizer2 = new RDXROS1PointCloudVisualizer("Obstacle point cloud", "/downsampled_cloud_obstacle");
      ros1PointCloudVisualizer2.subscribe(ros1Node);

      RDXROS1BoxVisualizer boxVisualizer = new RDXROS1BoxVisualizer("Object boxes", "/boxes");
      boxVisualizer.subscribe(ros1Node);

      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));


            ros1PointCloudVisualizer.create();
            ros1PointCloudVisualizer.setActive(true);
            ros1PointCloudVisualizer2.create();
            ros1PointCloudVisualizer2.setActive(true);

            //            ros1BoxVisualizer.create();
//            ros1BoxVisualizer.setEnabled(true);

            baseUI.getPrimaryScene().addRenderableProvider(ros1PointCloudVisualizer);
            baseUI.getPrimaryScene().addRenderableProvider(ros1PointCloudVisualizer2);
//            baseUI.getPrimaryScene().addRenderableProvider(ros1BoxVisualizer);

            baseUI.getPrimaryScene().addRenderableProvider(boxVisualizer);

//            baseUI.getPrimaryScene().addRenderableProvider(ros1BoxVisualizer);

            ros1Node.execute();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            boolean tunningParamsChanged = false;
            ImGui.begin(ImGuiTools.uniqueLabel(this, "Tuning"));
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
//            System.out.println("(double) tunableParameter1.getData()[0]"+(double) tunableParameter1.getData()[0]);
//            System.out.println("(double) tunableParameter2.getData()[0]"+(double) tunableParameter2.getData()[0]);
//            System.out.println("(double) tunableParameter3.getData()[0]"+(double) tunableParameter3.getData()[0]);
//            System.out.println("(double) tunableParameter4.getData()[0]"+(double) tunableParameter4.getData()[0]);
//            System.out.println("(double) tunableParameter5.getData()[0]"+(double) tunableParameter5.getData()[0]);
//            System.out.println("(double) tunableParameter6.getData()[0]"+(double) tunableParameter6.getData()[0]);
//            System.out.println("(double) tunableParameter7.getData()[0]"+(double) tunableParameter7.getData()[0]);
//            System.out.println("(double) tunableParameter8.getData()[0]"+(double) tunableParameter8.getData()[0]);
//            System.out.println("(double) tunableParameter9.getData()[0]"+(double) tunableParameter9.getData()[0]);

//            rosTunningParamPublisher.publish(0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1);

//            rosTunningParamPublisher.publish((double) tunableParameter1.getData()[0], (double) tunableParameter2.getData()[0], (double) tunableParameter3.getData()[0],
//                                                (double) tunableParameter4.getData()[0], (double) tunableParameter5.getData()[0], (double) tunableParameter6.getData()[6],
//                                                (double) tunableParameter7.getData()[0], (double) tunableParameter8.getData()[0], (double) tunableParameter9.getData()[0]);

//            }
//            System.out.println("tunableParameter1.getData()[0]:"+tunableParameter1.getData()[0]);
//            rosTunningParamPublisher.publish((double) tunableParameter1.getData()[0], (double) tunableParameter2.getData()[0], (double) tunableParameter3.getData()[0],
//                                             (double) tunableParameter4.getData()[0], (double) tunableParameter5.getData()[0], (double) tunableParameter6.getData()[6],
//                                             (double) tunableParameter7.getData()[0], (double) tunableParameter8.getData()[0], (double) tunableParameter9.getData()[0]);

            ros1PointCloudVisualizer.updateMesh(0.0f);
            ros1PointCloudVisualizer2.updateMesh(1.0f);
            boxVisualizer.update();

            //            ros1BoxVisualizer.updateMesh();

            baseUI.renderEnd();

         }
         @Override
         public void dispose()
         {
            boxVisualizer.dispose();
            baseUI.dispose();
         }

      }, APPLICATION_NAME, 1100, 1000);
   }


   public static void main(String[] args)
   {
      new RDXObstacleDetectionUI();
   }
}

