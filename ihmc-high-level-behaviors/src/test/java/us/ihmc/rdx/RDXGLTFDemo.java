package us.ihmc.rdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.math.Matrix4;
import imgui.ImGui;
import imgui.type.ImFloat;
import net.mgsx.gltf.loaders.gltf.GLTFLoader;
import net.mgsx.gltf.scene3d.scene.SceneAsset;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.LibGDXModelCopier;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXGLTFDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat currentScale = new ImFloat(10.0f);
   private final ImFloat opacity = new ImFloat(1.0f);
   private Model model;
   ModelInstance modelInstance;

   public RDXGLTFDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());

            Model load = RDXModelLoader.load("environmentObjects/flatGround/FlatGround.g3dj");
            load = LibGDXModelCopier.deepCopy(load);
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(load));

            FileHandle fileHandle = Gdx.files.internal("models/BoomBox.gltf");
            SceneAsset sceneAsset = new GLTFLoader().load(fileHandle, true);
            model = sceneAsset.scene.model;

            model.nodes.get(0).scale.set(currentScale.get(), currentScale.get(), currentScale.get());

            modelInstance = new ModelInstance(model);

            RigidBodyTransform transform = new RigidBodyTransform();
            transform.getRotation().setYawPitchRoll(0.0, 0.0, (float) Math.PI / 2.0);
            transform.getTranslation().set(0.2, 0.2, 0.2);
            LibGDXTools.toLibGDX(transform, modelInstance.transform);

            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool) -> modelInstance.getRenderables(renderables, pool));

            Model zedModel = RDXModelLoader.load("environmentObjects/ZEDMini/ZEDMini.glb");
            float zedScale = 1.0f;
            zedModel.nodes.get(0).scale.set(zedScale, zedScale, zedScale);
            RDXModelInstance zedInstance = new RDXModelInstance(zedModel);
            zedInstance.setPositionInWorldFrame(new Point3D(1.0, 1.0, 1.0));
            baseUI.getPrimaryScene().addModelInstance(zedInstance);

            Model rsModel = RDXModelLoader.load("environmentObjects/d435Sensor/D435.g3dj");
            float rsScale = 5.0f;
            rsModel.nodes.get(0).scale.set(rsScale, rsScale, rsScale);
            RDXModelInstance rsInstance = new RDXModelInstance(rsModel);
            rsInstance.setPositionInWorldFrame(new Point3D(-1.0, 1.0, 1.0));
            baseUI.getPrimaryScene().addModelInstance(rsInstance);

            baseUI.getImGuiPanelManager().addPanel("Settings", RDXGLTFDemo.this::renderImGuiWidgets);
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
      if (ImGui.sliderFloat(labels.get("Scale"), currentScale.getData(), 1.0f, 50.0f))
      {
         model.nodes.get(0).scale.set(currentScale.get(), currentScale.get(), currentScale.get());

         Matrix4 transform = modelInstance.transform;
         modelInstance = new ModelInstance(model);
         modelInstance.transform.set(transform);
      }

      if (ImGui.sliderFloat(labels.get("Opacity"), opacity.getData(), 0.0f, 1.0f))
      {
         for (Material material : modelInstance.materials)
         {
            if (opacity.get() < 1.0f)
               material.set(new BlendingAttribute(true, opacity.get()));
            else
               material.remove(BlendingAttribute.Type);
         }
      }
   }

   public static void main(String[] args)
   {
      new RDXGLTFDemo();
   }
}
