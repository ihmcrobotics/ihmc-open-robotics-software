package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRFootstep
{
   private final RobotSide side;
   private final ModelInstance modelInstance;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D pose = new FramePose3D();
   private RDX3DSituatedText footstepText;
   private final int index;
   private final FramePose3D textFramePose = new FramePose3D();

   public RDXVRFootstep(RobotSide side, ModelInstance modelInstance, int index)
   {
      this.side = side;
      this.index = index;
      this.modelInstance = modelInstance;
      LibGDXTools.setOpacity(this.modelInstance, 1.0f);

      LibGDXTools.toEuclid(modelInstance.transform, tempTransform);
      pose.set(tempTransform);
      // Somehow needed to have the solePose of the controller match the transform of the model instance
      pose.appendTranslation(new Point3D(0.01, 0.0, 0.01));

      String text = getSideNameFirstLetter(side);
      float textHeight = 0.08f;
      footstepText = new RDX3DSituatedText(text, textHeight);
   }

   public void setPose(FramePose3DReadOnly poseToSetInWorld)
   {
      pose.changeFrame(ReferenceFrame.getWorldFrame());
      pose.setToZero();
      pose.set(poseToSetInWorld);
      pose.get(tempTransform);

      LibGDXTools.toLibGDX(tempTransform, modelInstance.transform);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public RDX3DSituatedText getTextLabel()
   {
      textFramePose.setIncludingFrame(pose);
      textFramePose.appendYawRotation(-Math.PI / 2.0);
      textFramePose.appendTranslation(-0.04, 0.0, 0.035); // The text is higher in Z direction so it's not inside the foot
      textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      LibGDXTools.toLibGDX(textFramePose, tempTransform, footstepText.getModelTransform());
      return footstepText;
   }

   public String getSideNameFirstLetter(RobotSide side)
   {
      if (side == RobotSide.RIGHT)
         return "R" + index;
      else
         return "L" + index;
   }

   public RobotSide getSide()
   {
      return side;
   }

   public Pose3DReadOnly getPose()
   {
      return pose;
   }
}
