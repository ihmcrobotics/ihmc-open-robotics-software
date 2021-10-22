package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.openvr.*;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

public class GDXVRHandPlacedFootstepMode
{
   private final ArrayList<ModelInstance> footModels = new ArrayList<>();
   private final SideDependentList<ModelInstance> feetBeingPlaced = new SideDependentList<>();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D poseForPlacement = new FramePose3D();

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
               poseForPlacement.setToZero(controller.getXForwardZUpControllerFrame());
               poseForPlacement.getPosition().add(0.05, 0.0, 0.0);
               poseForPlacement.getOrientation().appendPitchRotation(Math.toRadians(-90.0));
               poseForPlacement.changeFrame(ReferenceFrame.getWorldFrame());
               poseForPlacement.get(tempTransform);

               GDXTools.toGDX(tempTransform, footBeingPlaced.transform);
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
