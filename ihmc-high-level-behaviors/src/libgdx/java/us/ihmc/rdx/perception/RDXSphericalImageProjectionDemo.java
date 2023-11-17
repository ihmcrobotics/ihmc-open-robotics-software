package us.ihmc.rdx.perception;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXSphericalImageProjectionDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final SideDependentList<RDXProjectionSphere> projectionSpheres = new SideDependentList<>(RDXProjectionSphere::new);
   private RDXIconTexture imageTexture;

   public RDXSphericalImageProjectionDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            imageTexture = new RDXIconTexture("/images/blackflytest.jpg");

            projectionSpheres.get(RobotSide.LEFT).create();
            projectionSpheres.get(RobotSide.LEFT).updateTexture(imageTexture.getTexture());

            baseUI.getPrimaryScene().addRenderableProvider(projectionSpheres.get(RobotSide.LEFT)::getRenderables);

            baseUI.getImGuiPanelManager().addPanel("Projection", () ->
            {
               projectionSpheres.get(RobotSide.LEFT).renderImGuiWidgets();
            });
         }

         @Override
         public void render()
         {
            projectionSpheres.get(RobotSide.LEFT).updateTexture(imageTexture.getTexture());

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
