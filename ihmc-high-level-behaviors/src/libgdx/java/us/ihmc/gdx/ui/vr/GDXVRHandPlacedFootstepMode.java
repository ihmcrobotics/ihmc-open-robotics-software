package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class GDXVRHandPlacedFootstepMode
{
   private final ArrayList<ModelInstance> footModels = new ArrayList<>();
   private final SideDependentList<ModelInstance> feetBeingPlaced = new SideDependentList<>();

   public void processVRInput(GDXVRContext vrContext)
   {
      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            InputDigitalActionData triggerClick = controller.getClickTriggerActionData();

            if (triggerClick.bChanged() && triggerClick.bState())
            {
               Model footModel = GDXModelLoader.loadG3DModel(side.getSideNameFirstLowerCaseLetter() + "_foot.g3dj");
               ModelInstance footModelInstance = new ModelInstance(footModel);
               footModels.add(footModelInstance);
               feetBeingPlaced.put(side, footModelInstance);
            }

            if (triggerClick.bChanged() && !triggerClick.bState())
            {
               feetBeingPlaced.put(side, null);
            }

            ModelInstance footBeingPlaced = feetBeingPlaced.get(side);
            if (footBeingPlaced != null)
            {
               controller.getGDXPoseInFrame(ReferenceFrame.getWorldFrame(), footBeingPlaced.transform);
            }
         });
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance footModel : footModels)
      {
         footModel.getRenderables(renderables, pool);
      }
   }
}
