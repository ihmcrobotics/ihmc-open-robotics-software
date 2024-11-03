package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class RDXFootstepGraphicDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Footstep Graphic Demo");
   private final RDXFootstepPlanGraphic footstepPlanGraphic = new RDXFootstepPlanGraphic();

   public RDXFootstepGraphicDemo()
   {
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
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
            footsteps.add(new MinimalFootstep(RobotSide.LEFT, leftPose, foothold, "Left"));
            Pose3D rightPose = new Pose3D();
            rightPose.setY(-1.0);
            footsteps.add(new MinimalFootstep(RobotSide.RIGHT, rightPose, foothold, "Right"));
            footstepPlanGraphic.generateMeshes(footsteps);

            baseUI.getPrimaryScene().addRenderableProvider(footstepPlanGraphic);
         }

         @Override
         public void render()
         {
            footstepPlanGraphic.update();

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
      new RDXFootstepGraphicDemo();
   }
}
