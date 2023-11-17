package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Set;

public class RDXSphericalImageProjectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(RDXProjectionSphere::new);
   private final SideDependentList<RDXIconTexture> imageTextures = new SideDependentList<>();

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
               projectionSpheres.get(side).updateTexture(imageTextures.get(side).getTexture());
            }

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection", () ->
            {
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
               baseUI.getPrimaryScene().addRenderableProvider(projectionSpheres.get(RobotSide.RIGHT)::getRenderables);
            }
            else
            {
               baseUI.getPrimaryScene().addRenderableProvider(projectionSpheres.get(RobotSide.LEFT)::getRenderables);
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
