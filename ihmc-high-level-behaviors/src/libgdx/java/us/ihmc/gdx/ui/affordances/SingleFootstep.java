package us.ihmc.gdx.ui.affordances;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.sceneManager.GDX3DScene;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;

public class SingleFootstep
{
   private GDXModelInstance footstepModelInstance;

   public enum FootstepSide {LEFT, RIGHT, NONE};
   private FootstepSide footstepSide = FootstepSide.NONE;
   private GDX3DScene primaryScene;
   ReferenceFrame referenceFrameFootstep;

   public SingleFootstep(GDX3DScene primaryScene, FootstepSide footstepSide)
   {
      this.footstepSide = footstepSide;
      this.primaryScene = primaryScene;

      if (footstepSide.equals(FootstepSide.LEFT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_left.g3dj"));
      }
      else if (footstepSide.equals(FootstepSide.LEFT))
      {

      }

      primaryScene.addModelInstance(footstepModelInstance, GDXSceneLevel.VIRTUAL);
      Pose3D pose = new Pose3D();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      referenceFrameFootstep = ReferenceFrameTools.constructFrameWithChangingTransformToParent("footstep frame",
                                                                                               ReferenceFrame.getWorldFrame(), rigidBodyTransform);

      pose.get(rigidBodyTransform);

      referenceFrameFootstep.update();
   }
}
