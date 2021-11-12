package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXVRHandPlacedFootstep
{
   private final RobotSide side;
   private final ModelInstance modelInstance;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Pose3D solePose = new Pose3D();

   public GDXVRHandPlacedFootstep(RobotSide side, ModelInstance modelInstance, RigidBodyTransform soleToAnkleTransform)
   {
      this.side = side;
      this.modelInstance = modelInstance;

      GDXTools.toEuclid(modelInstance.transform, tempTransform);
      solePose.set(tempTransform);
      solePose.appendTransform(soleToAnkleTransform);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public Pose3DReadOnly getSolePose()
   {
      return solePose;
   }
}
