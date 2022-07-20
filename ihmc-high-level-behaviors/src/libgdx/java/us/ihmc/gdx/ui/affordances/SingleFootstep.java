package us.ihmc.gdx.ui.affordances;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.sceneManager.GDX3DScene;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.robotics.robotSide.RobotSide;

public class SingleFootstep
{
   private GDXModelInstance footstepModelInstance;

   private RobotSide footstepSide;
   ReferenceFrame referenceFrameFootstep;

   public Pose3D footPose;

   public SingleFootstep(GDXImGuiBasedUI baseUI, RobotSide footstepSide)
   {
      this.footstepSide = footstepSide;
      if (footstepSide.equals(RobotSide.LEFT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_left.g3dj"));
      }
      else if (footstepSide.equals(RobotSide.RIGHT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_right.g3dj"));
      }

      baseUI.getPrimaryScene().addModelInstance(footstepModelInstance, GDXSceneLevel.VIRTUAL);


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

   public void setFootstepModelInstance(GDXModelInstance footstepModelInstance)
   {
      this.footstepModelInstance = footstepModelInstance;
   }

   public RobotSide getFootstepSide()
   {
      return footstepSide;
   }
   public GDXModelInstance getFootstepModelInstance()
   {
      return footstepModelInstance;
   }
}
