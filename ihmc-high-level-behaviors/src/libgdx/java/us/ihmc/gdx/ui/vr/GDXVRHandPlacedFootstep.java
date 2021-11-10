package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXVRHandPlacedFootstep
{
   private final RobotSide side;
   private final ModelInstance modelInstance;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D pose = new FramePose3D();

   public GDXVRHandPlacedFootstep(RobotSide side, ModelInstance modelInstance)
   {
      this.side = side;
      this.modelInstance = modelInstance;

      GDXTools.toEuclid(modelInstance.transform, tempTransform);
      pose.setIncludingFrame(ReferenceFrame.getWorldFrame(), tempTransform);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public FramePose3D getPose()
   {
      return pose;
   }
}
