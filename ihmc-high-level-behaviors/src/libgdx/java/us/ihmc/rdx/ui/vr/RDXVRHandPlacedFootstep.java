package us.ihmc.rdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRHandPlacedFootstep
{
   private final RobotSide side;
   private final ModelInstance modelInstance;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final FramePose3D solePose = new FramePose3D();
   private RDX3DSituatedText footstepText;
   private final int index;
   private final FramePose3D textFramePose = new FramePose3D();

   public RDXVRHandPlacedFootstep(RobotSide side, ModelInstance modelInstance, int index, RigidBodyTransform soleToAnkleTransform)
   {
      this.side = side;
      this.index = index;
      this.modelInstance = modelInstance;
      LibGDXTools.setOpacity(this.modelInstance, 1.0f);

      LibGDXTools.toEuclid(modelInstance.transform, tempTransform);
      solePose.set(tempTransform);
      // Somehow needed to have the solePose of the controller match the transform of the model instance
      solePose.appendTranslation(new Point3D(0.01, 0.0, 0.01));
      solePose.appendTransform(soleToAnkleTransform);

      String text = getSideNameFirstLetter(side);
      float textHeight = 0.08f;
      footstepText = new RDX3DSituatedText(text, textHeight);
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public RDX3DSituatedText getFootstepText()
   {
      textFramePose.setIncludingFrame(solePose);
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

   public Pose3DReadOnly getSolePose()
   {
      return solePose;
   }
}
