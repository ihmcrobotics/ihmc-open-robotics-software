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
   ReferenceFrame referenceFrameFootstep;

   public Pose3D footPose;

   public SingleFootstep(GDX3DScene primaryScene, FootstepSide footstepSide)
   {
      this.footstepSide = footstepSide;
      if (footstepSide.equals(FootstepSide.LEFT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_left.g3dj"));
      }
      else if (footstepSide.equals(FootstepSide.RIGHT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_right.g3dj"));
      }

      primaryScene.addModelInstance(footstepModelInstance, GDXSceneLevel.VIRTUAL);
      Pose3D pose = new Pose3D();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      referenceFrameFootstep = ReferenceFrameTools.constructFrameWithChangingTransformToParent("footstep frame",
                                                                                               ReferenceFrame.getWorldFrame(), rigidBodyTransform);

      pose.get(rigidBodyTransform);

      footPose = pose;

      referenceFrameFootstep.update();
   }

   public void setFootPose(double x, double y, double z)
   {
      this.footPose.setX(x);
      this.footPose.setY(y);
      this.footPose.setZ(z);

   }

   public FootstepSide getFootstepSide()
   {
      return footstepSide;
   }
   public GDXModelInstance getFootstepModelInstance()
   {
      return footstepModelInstance;
   }
}
