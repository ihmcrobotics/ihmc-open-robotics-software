package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXGLTFDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiSliderDoubleWrapper ambientLightIntensitySlider;
   private ImGuiSliderDoubleWrapper pointLightIntensitySlider;
   private ImGuiSliderDoubleWrapper directionalLightIntensitySlider;

   public RDXGLTFDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            ModelData d455SensorModel = RDXModelLoader.loadModelData("environmentObjects/d455Sensor/D455.g3dj");
            Model model = new Model(d455SensorModel);
            RDXModelInstance modelInstance = new RDXModelInstance(model);
            modelInstance.setPoseInWorldFrame(new Pose3D(0.5, 0.5, 0.5, 0.0, 0.0, 0.0));
            baseUI.getPrimaryScene().addModelInstance(modelInstance);

            ModelData blackflyModel = RDXModelLoader.loadModelData("environmentObjects/blackflyFujinon/BlackflyFujinon.g3dj");
            model = new Model(blackflyModel);
            modelInstance = new RDXModelInstance(model);
            modelInstance.setPoseInWorldFrame(new Pose3D(0.5, -0.5, 0.5, 0.0, 0.0, 0.0));
            baseUI.getPrimaryScene().addModelInstance(modelInstance);

            baseUI.getImGuiPanelManager().addPanel("Settings", RDXGLTFDemo.this::renderImGuiWidgets);

            ambientLightIntensitySlider = new ImGuiSliderDoubleWrapper("Ambient light intensity", "%.2f", 0.0, 0.05,
                                                                       () -> baseUI.getPrimaryScene().getAmbientLightIntensity(),
                                                                       value -> baseUI.getPrimaryScene().setAmbientLightIntensity((float) value));
            pointLightIntensitySlider = new ImGuiSliderDoubleWrapper("Point light intensity", "%.1f", 0.0, 1000.0,
                                                                       () -> baseUI.getPrimaryScene().getPointLightIntensity(),
                                                                       value -> baseUI.getPrimaryScene().setPointLightIntensity((float) value));
            directionalLightIntensitySlider = new ImGuiSliderDoubleWrapper("Directional light intensity", "%.1f", 0.0, 10.0,
                                                                           () -> baseUI.getPrimaryScene().getDirectionalLightIntensity(),
                                                                           value -> baseUI.getPrimaryScene().setDirectionalLightIntensity((float) value));
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   private void renderImGuiWidgets()
   {
      ambientLightIntensitySlider.renderImGuiWidget();
      pointLightIntensitySlider.renderImGuiWidget();
      directionalLightIntensitySlider.renderImGuiWidget();
   }

   public static void main(String[] args)
   {
      new RDXGLTFDemo();
   }
}
