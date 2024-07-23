package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.g3d.utils.shapebuilders.BoxShapeBuilder;
import net.mgsx.gltf.scene3d.attributes.PBRColorAttribute;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.BoxesDemoModel;
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

//            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            ModelBuilder mb = new ModelBuilder();
            mb.begin();
            Material material = new Material();
            material.set(PBRColorAttribute.createBaseColorFactor(new Color(Color.WHITE).fromHsv(15, .9f, .8f)));
            MeshPartBuilder mpb = mb.part("cube", GL20.GL_TRIANGLES, Usage.Position | Usage.Normal, material);
            BoxShapeBuilder.build(mpb, 1f, 1f, 1f);
            Model model = mb.end();
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(model));

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
