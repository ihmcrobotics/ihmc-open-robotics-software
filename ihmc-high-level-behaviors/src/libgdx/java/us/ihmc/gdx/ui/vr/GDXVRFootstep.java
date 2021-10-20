package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXVRFootstep
{
   private final ModelInstance modelInstance;
   private RobotSide side;

   public GDXVRFootstep(RobotSide side)
   {
      this.side = side;
      Model model = GDXModelLoader.loadG3DModel(side.getSideNameFirstLowerCaseLetter() + "_foot.g3dj");
      modelInstance = new ModelInstance(model);
   }
}
