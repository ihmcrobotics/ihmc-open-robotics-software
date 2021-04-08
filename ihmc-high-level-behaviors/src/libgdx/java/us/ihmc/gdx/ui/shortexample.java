package us.ihmc.gdx.ui;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.live.GDXROS1PointCloudVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;


public class shortexample
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI();
   public shortexample()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "ouster_point_cloud_viewer");

      GDXROS1PointCloudVisualizer ros1PointCloudVisualizer = new GDXROS1PointCloudVisualizer(ros1Node,
                                                                                             "/downsampled_cloud_road",
                                                                                             ReferenceFrame.getWorldFrame(),
                                                                                             new RigidBodyTransform());

      GDXROS1PointCloudVisualizer ros1PointCloudVisualizer2 = new GDXROS1PointCloudVisualizer(ros1Node,
                                                                                              "/downsampled_cloud_obstacle",
                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                              new RigidBodyTransform());
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));

            ros1PointCloudVisualizer.create();
            ros1PointCloudVisualizer.setEnabled(true);
            ros1PointCloudVisualizer2.create();
            ros1PointCloudVisualizer2.setEnabled(true);

            baseUI.getSceneManager().addRenderableProvider(ros1PointCloudVisualizer);
            baseUI.getSceneManager().addRenderableProvider(ros1PointCloudVisualizer2);


            ros1Node.execute();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();

            ImGui.end();

            ros1PointCloudVisualizer.updateMeshcolor(0.0f);
            ros1PointCloudVisualizer2.updateMeshcolor(1.0f);

            baseUI.renderEnd();

         }

      }, getClass().getSimpleName(), 1100, 800);
   }

   public static void main(String[] args)
   {
      new shortexample();
   }
}
