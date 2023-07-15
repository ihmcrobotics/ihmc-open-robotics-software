package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXVRControllerButtonLabel
{
   private ModifiableReferenceFrame textFrame;
   private RDX3DSituatedText situated3DText;

   public RDXVRControllerButtonLabel(ReferenceFrame controllerFrame, RobotSide side, Point3D labelOffset)
   {
      textFrame = new ModifiableReferenceFrame(controllerFrame);
      textFrame.update(transformToParent ->
      {
         transformToParent.getTranslation().set(labelOffset);
         EuclidCoreMissingTools.setYawPitchRollDegrees(transformToParent.getRotation(),
                                                       -90.0 + side.negateIfLeftSide(10.0),
                                                       0.0,
                                                       15.0);
      });
   }

   public void setText(String text)
   {
      situated3DText = new RDX3DSituatedText(text, RDX3DSituatedText.DEFAULT_FONT, java.awt.Color.WHITE);
   }

   public void update()
   {
      LibGDXTools.toLibGDX(textFrame.getReferenceFrame().getTransformToRoot(), situated3DText.getModelInstance().transform);
      situated3DText.scale(0.01f);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (situated3DText != null)
      {
         situated3DText.getRenderables(renderables, pool);
      }
   }
}
