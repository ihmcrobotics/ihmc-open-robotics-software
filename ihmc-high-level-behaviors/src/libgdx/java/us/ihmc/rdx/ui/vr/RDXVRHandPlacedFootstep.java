package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRHandPlacedFootstep
{
   private final RobotSide side;
   private final ModelInstance modelInstance;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Pose3D solePose = new Pose3D();

   public RDXVRHandPlacedFootstep(RobotSide side, ModelInstance modelInstance, RigidBodyTransform soleToAnkleTransform)
   {
      this.side = side;
      this.modelInstance = modelInstance;

      LibGDXTools.toEuclid(modelInstance.transform, tempTransform);
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
