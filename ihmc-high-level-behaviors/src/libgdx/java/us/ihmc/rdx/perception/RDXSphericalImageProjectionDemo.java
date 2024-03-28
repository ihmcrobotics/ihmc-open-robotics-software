package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImDouble;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public class RDXSphericalImageProjectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(/*RDXProjectionSphere::new*/);
   private final SideDependentList<RDXIconTexture> imageTextures = new SideDependentList<>();
   private final ImDouble pupillaryDistance = new ImDouble(0.180724);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final RigidBodyTransform leftEyePose = new RigidBodyTransform();

   public RDXSphericalImageProjectionDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            for (RobotSide side : RobotSide.values)
            {
               imageTextures.put(side, new RDXIconTexture("/images/blackflytest%s.jpg".formatted(side.getLowerCaseName())));
               projectionSpheres.get(side).create();
               projectionSpheres.get(side).updateTexture(imageTextures.get(side).getTexture(), 1.0f);
            }

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection", () ->
            {
               if (ImGuiTools.sliderDouble(labels.get("Pupillary distance"), pupillaryDistance, 0.1, 0.85))
               {
                  leftEyePose.getTranslation().setY(pupillaryDistance.get());
                  LibGDXTools.toLibGDX(leftEyePose, projectionSpheres.get(RobotSide.LEFT).getModelInstance().transform);
               }
               ImGui.text("Left:");
               projectionSpheres.get(RobotSide.LEFT).renderImGuiWidgets();
               ImGui.text("Right:");
               projectionSpheres.get(RobotSide.RIGHT).renderImGuiWidgets();
            });
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
         {
            if (sceneLevels.contains(RDXSceneLevel.VR_EYE_RIGHT))
            {
               projectionSpheres.get(RobotSide.RIGHT).getRenderables(renderables, pool);
            }
            else
            {
               projectionSpheres.get(RobotSide.LEFT).getRenderables(renderables, pool);
            }
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

   public static void main(String[] args)
   {
      new RDXSphericalImageProjectionDemo();
   }
}
