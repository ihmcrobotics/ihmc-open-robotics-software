package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRControllerButtonLabel
{
   private final MutableReferenceFrame textFrame;
   private final RDX3DSituatedText situatedText;

   public RDXVRControllerButtonLabel(ReferenceFrame controllerFrame, RobotSide side, Point3D labelOffset, YawPitchRoll labelOrientation)
   {
      textFrame = new MutableReferenceFrame(controllerFrame);
      textFrame.update(transformToParent ->
      {
         transformToParent.getTranslation().set(labelOffset);
         EuclidCoreMissingTools.setYawPitchRollDegrees(transformToParent.getRotation(),
                                                       -90.0 + side.negateIfLeftSide(10.0),
                                                       0.0,
                                                       15.0);
         transformToParent.getRotation().append(labelOrientation);
      });

      situatedText = new RDX3DSituatedText("", java.awt.Color.WHITE, 0.01f);
   }

   public void setText(String text)
   {
      situatedText.setText(text);
      LibGDXTools.toLibGDX(textFrame.getReferenceFrame().getTransformToRoot(), situatedText.getModelTransform());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      situatedText.getRenderables(renderables, pool);
   }
}
