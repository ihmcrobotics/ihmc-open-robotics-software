package us.ihmc.rdx.vr;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.openvr.InputAnalogActionData;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * A radial menu showing 4 options. A white sphere shows above the joystick
 * when the menu is available. Move the joystick in a direction to see the options.
 * Click and release the joystick button while holding it in the direction of
 * the desired option to execute the option. A white mesh box highlights the
 * selected option.
 *
 * TODO: It's a little awkward to press the joystick button while holding it in
 *   a direction, so we could look into using the trigger to do that or
 *   something.
 */
public class RDXVRControllerRadialMenu
{
   private final RobotSide side;
   private final FrameBox3D selectionCollisionBox = new FrameBox3D();
   private final MutableReferenceFrame joystickReferenceFrame;
   private final MutableReferenceFrame radialMenuSpherePoseFrame;
   private final MutableReferenceFrame radialMenuReferenceFrame;
   private InputAnalogActionData joystickActionData;
   private InputDigitalActionData joystickPressActionData;
   private RDXModelInstance radialMenuSphere;
   private RDXModelInstance radialMenuSelectionGraphic;
   private boolean showSphere = false;
   private boolean showSelectionBox = false;
   private RDXVRRadialMenuSelection radialMenuSelection = RDXVRRadialMenuSelection.NONE;
   private Point3D topJoystickOffset;
   private Point3D bottomJoystickOffset;
   private Point3D leftJoystickOffset;
   private Point3D rightJoystickOffset;
   private RDXVRControllerButtonLabel topJoystickLabel;
   private RDXVRControllerButtonLabel bottomJoystickLabel;
   private RDXVRControllerButtonLabel leftJoystickLabel;
   private RDXVRControllerButtonLabel rightJoystickLabel;
   private final Point3D boxOffset = new Point3D();
   private final FramePose3D radialMenuFramePose = new FramePose3D();
   private final FramePose3D joystickFramePose = new FramePose3D();
   private final FramePose3D joystickSphereFramePose = new FramePose3D();

   public RDXVRControllerRadialMenu(RobotSide side, ReferenceFrame xForwardZUpControllerFrame)
   {
      this.side = side;

      selectionCollisionBox.getSize().set(0.0125, 0.075, 0.0025);

      joystickReferenceFrame = new MutableReferenceFrame(xForwardZUpControllerFrame);
      joystickReferenceFrame.getTransformToParent().getTranslation().setX(-0.04);
      joystickReferenceFrame.getTransformToParent().getTranslation().setY(side.negateIfLeftSide(-0.015));
      joystickReferenceFrame.getTransformToParent().getTranslation().setZ(-0.017);
      joystickReferenceFrame.getReferenceFrame().update();

      radialMenuSpherePoseFrame = new MutableReferenceFrame(joystickReferenceFrame.getReferenceFrame());
      radialMenuReferenceFrame = new MutableReferenceFrame(joystickReferenceFrame.getReferenceFrame());
   }

   public void create(InputAnalogActionData joystickActionData, InputDigitalActionData joystickPressActionData)
   {
      this.joystickActionData = joystickActionData;
      this.joystickPressActionData = joystickPressActionData;

      radialMenuSphere = new RDXModelInstance(RDXModelBuilder.createSphere(0.0025f, new Color(Color.WHITE)));
      radialMenuSelectionGraphic = new RDXModelInstance(RDXModelBuilder.buildModel(
            boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(selectionCollisionBox.getVertices(), 0.0005, new Color(Color.WHITE))));

      topJoystickOffset = new Point3D(0.1, 0.0, 0.0);
      bottomJoystickOffset = new Point3D(-0.1, 0.0, -0.006);
      rightJoystickOffset = new Point3D(0.0, -0.1, -0.002);
      leftJoystickOffset = new Point3D(0.0, 0.1, -0.002);
      topJoystickLabel = new RDXVRControllerButtonLabel(joystickReferenceFrame.getReferenceFrame(), side, topJoystickOffset, new YawPitchRoll());
      bottomJoystickLabel = new RDXVRControllerButtonLabel(joystickReferenceFrame.getReferenceFrame(), side, bottomJoystickOffset, new YawPitchRoll());
      rightJoystickLabel = new RDXVRControllerButtonLabel(joystickReferenceFrame.getReferenceFrame(), side, rightJoystickOffset, new YawPitchRoll());
      leftJoystickLabel = new RDXVRControllerButtonLabel(joystickReferenceFrame.getReferenceFrame(), side, leftJoystickOffset, new YawPitchRoll());
   }

   public void update()
   {
      joystickReferenceFrame.getReferenceFrame().update();
      joystickFramePose.setToZero(joystickReferenceFrame.getReferenceFrame());
      joystickFramePose.changeFrame(ReferenceFrame.getWorldFrame());

      radialMenuSpherePoseFrame.getReferenceFrame().update();
      joystickSphereFramePose.setToZero(radialMenuSpherePoseFrame.getReferenceFrame());
      joystickSphereFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      radialMenuSphere.setPoseInWorldFrame(joystickSphereFramePose);

      topJoystickLabel.setText("");
      bottomJoystickLabel.setText("");
      leftJoystickLabel.setText("");
      rightJoystickLabel.setText("");

      double radialSelectorTranslationScalar = 0.1;
      radialMenuSpherePoseFrame.getTransformToParent().getTranslation().setX(radialSelectorTranslationScalar * joystickActionData.y());
      radialMenuSpherePoseFrame.getTransformToParent().getTranslation().setY(-radialSelectorTranslationScalar * joystickActionData.x());

      showSphere = false;
      showSelectionBox = false;
      radialMenuSelection = RDXVRRadialMenuSelection.NONE;
   }

   public void run(RDXVRController controller,
                   String topText, String bottomText, String rightText, String leftText,
                   Runnable topRunnable, Runnable bottomRunnable, Runnable rightRunnable, Runnable leftRunnable)
   {
      showSphere = true;

      if (!controller.getJoystickIsCentered())
      {
         showSelectionBox = true;

         topJoystickLabel.setText(topText);
         bottomJoystickLabel.setText(bottomText);
         rightJoystickLabel.setText(rightText);
         leftJoystickLabel.setText(leftText);

         if (joystickActionData.x() < 0 && Math.abs(joystickActionData.x()) > Math.abs(joystickActionData.y()))
         {
            radialMenuSelection = RDXVRRadialMenuSelection.LEFT_RING;
         }
         if (joystickActionData.x() > 0 && Math.abs(joystickActionData.x()) > Math.abs(joystickActionData.y()))
         {
            radialMenuSelection = RDXVRRadialMenuSelection.RIGHT_RING;
         }
         if (joystickActionData.y() > 0 && Math.abs(joystickActionData.y()) > Math.abs(joystickActionData.x()))
         {
            radialMenuSelection = RDXVRRadialMenuSelection.TOP_RING;
         }
         if (joystickActionData.y() < 0 && Math.abs(joystickActionData.y()) > Math.abs(joystickActionData.x()))
         {
            radialMenuSelection = RDXVRRadialMenuSelection.BOTTOM_RING;
         }

         switch (radialMenuSelection)
         {
            case LEFT_RING -> boxOffset.set(leftJoystickOffset);
            case RIGHT_RING -> boxOffset.set(rightJoystickOffset);
            case TOP_RING -> boxOffset.set(topJoystickOffset);
            case BOTTOM_RING -> boxOffset.set(bottomJoystickOffset);
         }

         radialMenuReferenceFrame.getReferenceFrame().getTransformToParent().getTranslation().set(boxOffset);
         radialMenuReferenceFrame.getReferenceFrame().update();
         radialMenuFramePose.setToZero(radialMenuReferenceFrame.getReferenceFrame());
         radialMenuFramePose.getTranslation().add(boxOffset);
         radialMenuFramePose.getTranslation().subY(0.03);
         if (side == RobotSide.RIGHT)
         {
            radialMenuFramePose.getTranslation().addX(0.01);
         }
         radialMenuFramePose.getRotation().setToYawOrientation(side.negateIfLeftSide(0.2));
         radialMenuFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         radialMenuSelectionGraphic.setPoseInWorldFrame(radialMenuFramePose);

         if (joystickPressActionData.bChanged() && !joystickPressActionData.bState())
         {
            switch (radialMenuSelection)
            {
               case TOP_RING -> topRunnable.run();
               case BOTTOM_RING -> bottomRunnable.run();
               case RIGHT_RING -> rightRunnable.run();
               case LEFT_RING -> leftRunnable.run();
            }
         }
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showSphere)
         radialMenuSphere.getRenderables(renderables, pool);
      if (showSelectionBox)
         radialMenuSelectionGraphic.getRenderables(renderables, pool);

      if (topJoystickLabel != null)
         topJoystickLabel.getRenderables(renderables, pool);
      if (bottomJoystickLabel != null)
         bottomJoystickLabel.getRenderables(renderables, pool);
      if (rightJoystickLabel != null)
         rightJoystickLabel.getRenderables(renderables, pool);
      if (leftJoystickLabel != null)
         leftJoystickLabel.getRenderables(renderables, pool);
   }
}
