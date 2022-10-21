package us.ihmc.rdx.tools;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class EuclidModelInstance extends ModelInstance
{
   private final Pose3D userPose = new Pose3D();
   private final Pose3D tempPose = new Pose3D();
   private final RigidBodyTransform tempRigidBodyTransform = new RigidBodyTransform();

   private final RigidBodyTransform userTransform = new RigidBodyTransform();
   private final RigidBodyTransform offsetTransform = new RigidBodyTransform();

   public EuclidModelInstance(Model model, Pose3D offsetPose)
   {
      super(model);

      offsetPose.get(offsetTransform);
   }

   public Pose3D getPoseToSet()
   {
      return userPose;
   }

   public void updatePose()
   {
      userPose.get(userTransform);
      tempPose.set(offsetTransform);
      tempPose.applyTransform(userTransform);
      tempPose.get(tempRigidBodyTransform);
      LibGDXTools.toGDX(tempRigidBodyTransform, transform);
   }
}
