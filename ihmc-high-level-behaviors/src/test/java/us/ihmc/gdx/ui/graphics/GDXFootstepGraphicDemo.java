package us.ihmc.gdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class GDXFootstepGraphicDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "Footstep Graphic Demo");
   private final GDXFootstepPlanGraphic footstepPlanGraphic = new GDXFootstepPlanGraphic();

   public GDXFootstepGraphicDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ArrayList<MinimalFootstep> footsteps = new ArrayList<>();
            Pose3D leftPose = new Pose3D();
            leftPose.setY(1.0);
            ConvexPolygon2D foothold = new ConvexPolygon2D();
            double halfLength = 0.1;
            double halfWidth = 0.05;
            foothold.addVertex(halfLength, -halfWidth);
            foothold.addVertex(halfLength, halfWidth);
            foothold.addVertex(-halfLength, halfWidth);
            foothold.addVertex(-halfLength, -halfWidth);
            foothold.update();
            footsteps.add(new MinimalFootstep(RobotSide.LEFT, leftPose, foothold));
            Pose3D rightPose = new Pose3D();
            rightPose.setY(-1.0);
            footsteps.add(new MinimalFootstep(RobotSide.RIGHT, rightPose, foothold));
            footstepPlanGraphic.generateMeshes(footsteps);

            baseUI.getSceneManager().addModelInstance(new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3)));
            baseUI.getSceneManager().addRenderableProvider(footstepPlanGraphic);
         }

         @Override
         public void render()
         {
            footstepPlanGraphic.render();

            baseUI.renderBeforeOnScreenUI();

            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      }, getClass());
   }

   public static void main(String[] args)
   {
      new GDXFootstepGraphicDemo();
   }
}
