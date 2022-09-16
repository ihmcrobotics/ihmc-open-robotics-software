package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.gdx.GDX3DSituatedText;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.input.ImGui3DViewPickResult;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.StepCheckIsPointInsideAlgorithm;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class GDXInteractableFootstep
{
   // Intended to reuse text renderables, as they are relatively expensive to create
   private static final Map<String, GDX3DSituatedText> textRenderablesMap = new HashMap<>();
   private GDX3DSituatedText footstepIndexText;
   private GDXModelInstance footstepModelInstance;
   private RobotSide footstepSide = null;
   private GDXSelectablePose3DGizmo selectablePose3DGizmo;
   private final FramePose3D tempFramePose = new FramePose3D();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private boolean isHovered;
   private final Sphere3D boundingSphere = new Sphere3D(0.1);
   private boolean isClickedOn;
   private final FramePose3D textFramePose = new FramePose3D();
   private final Timer timerFlashingFootsteps = new Timer();
   private boolean flashingFootStepsColorHigh = false;
   private final ImGui3DViewPickResult pickResult = new ImGui3DViewPickResult();

   public GDXInteractableFootstep(GDXImGuiBasedUI baseUI, RobotSide footstepSide, int index)
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

      selectablePose3DGizmo = new GDXSelectablePose3DGizmo();
      selectablePose3DGizmo.create(baseUI.getPrimary3DPanel());

      String txt = footstepSide.getSideNameFirstLetter() + (index + 1);
      if (!textRenderablesMap.containsKey(txt))
      {
         footstepIndexText = new GDX3DSituatedText("" + txt);
         textRenderablesMap.put(txt, footstepIndexText);
      }
      else
      {
         footstepIndexText = textRenderablesMap.get(txt);
      }
   }

   public GDXInteractableFootstep(GDXImGuiBasedUI baseUI, PlannedFootstep plannedFootstep, int footstepIndex)
   {
      updateFromPlannedStep(baseUI,plannedFootstep,footstepIndex);
   }

   public void updateFromPlannedStep(GDXImGuiBasedUI baseUI, PlannedFootstep plannedFootstep, int footstepIndex )
   {
      this.footstepSide = plannedFootstep.getRobotSide();
      if (footstepSide.equals(RobotSide.LEFT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_left.g3dj"));
      }
      else if (footstepSide.equals(RobotSide.RIGHT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_right.g3dj"));
      }

      selectablePose3DGizmo = new GDXSelectablePose3DGizmo();
      selectablePose3DGizmo.create(baseUI.getPrimary3DPanel());

      String text = footstepSide.getSideNameFirstLetter() + (footstepIndex + 1);
      if (!textRenderablesMap.containsKey(text))
      {
         footstepIndexText = new GDX3DSituatedText("" + text);
         textRenderablesMap.put(text, footstepIndexText);
      }
      else
      {
         footstepIndexText = textRenderablesMap.get(text);
      }

      updatePose(plannedFootstep.getFootstepPose());
   }

   public void update()
   {
      selectablePose3DGizmo.getPoseGizmo().getPose().get(tempTransform);
      double textHeight = 0.08;
      textFramePose.setToZero(selectablePose3DGizmo.getPoseGizmo().getPose().getReferenceFrame());
      textFramePose.set(selectablePose3DGizmo.getPoseGizmo().getPose());

      textFramePose.appendYawRotation(-Math.PI / 2.0);
      textFramePose.appendTranslation(-0.03, 0.0, 0.035); //note: Make text higher in z direction, so it's not inside the foot
      textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      GDXTools.toGDX(textFramePose, tempTransform, footstepIndexText.getModelInstance().transform);
      footstepIndexText.scale((float) textHeight);
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);

      StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();
      stepCheckIsPointInsideAlgorithm.setup(boundingSphere.getRadius(), boundingSphere.getPosition());

      Function<Point3DReadOnly, Boolean> isPointInside = boundingSphere::isPointInside;
      boolean pickIntersected = !Double.isNaN(stepCheckIsPointInsideAlgorithm.intersect(input.getPickRayInWorld(), 100, isPointInside));
      if (pickIntersected)
      {
         pickResult.setDistanceToCamera(stepCheckIsPointInsideAlgorithm.getClosestIntersection().distance(input.getPickRayInWorld().getPoint()));
         input.addPickResult(pickResult);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input, boolean currentlyPlacingFootstep)
   {
      isHovered = pickResult == input.getClosestPick();
      isClickedOn = isHovered && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);

      // TODO: mouse hovering on the footstep. (get foot validity warning text when this happens)
      if (isHovered)
      {
         if (footstepSide == RobotSide.LEFT)
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 1.0f, 0.0f, 0.0f, 0.0f));
         else
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.0f, 1.0f, 0.0f, 0.0f));
      }
      else
      {
         if (footstepSide == RobotSide.LEFT)
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.5f, 0.0f, 0.0f, 0.0f));
         else
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.0f, 0.5f, 0.0f, 0.0f));
      }

      // FIXME:
      if (currentlyPlacingFootstep)
      {
         selectablePose3DGizmo.process3DViewInput(input);
      }
      else
      {
         selectablePose3DGizmo.process3DViewInput(input, isHovered);
      }

//      selectablePose3DGizmo.process3DViewInput(input, isHovered);

      footstepModelInstance.transform.setToRotationRad(selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getX32(),
                                                       selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getY32(),
                                                       selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().getZ32(),
                                                       (float) selectablePose3DGizmo.getPoseGizmo().getPose().getRotation().angle());
      footstepModelInstance.transform.setTranslation(selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getX32(),
                                                     selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getY32(),
                                                     selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getZ32());
      boundingSphere.getPosition()
                    .set(selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getX32(),
                         selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getY32(),
                         selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getZ32());


   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
      footstepIndexText.getRenderables(renderables, pool);
      footstepModelInstance.getRenderables(renderables, pool);
   }

   // Sets the gizmo's position and rotation
   public void setGizmoPose(double x, double y, double z, RigidBodyTransform transform)
   {
      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      tempFramePose.getPosition().set(x, y, z);
      tempFramePose.getRotation().set(transform.getRotation());
      tempFramePose.get(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
   }

   public void flashFootstepWhenBadPlacement(BipedalFootstepPlannerNodeRejectionReason reason)
   {
      if (reason == null)
      {
         if (getFootstepSide() == RobotSide.LEFT)
         {
            if (isHovered())
               setColor(1.0f, 0.0f, 0.0f, 0.0f);
            else
               setColor(0.5f, 0.0f, 0.0f, 0.0f);
         }
         else
         {
            if (isHovered())
               setColor(0.0f, 1.0f, 0.0f, 0.0f);
            else
               setColor(0.0f, 0.5f, 0.0f, 0.0f);
         }
      }
      else
      {
         if (!timerFlashingFootsteps.hasBeenSet())
         {
            timerFlashingFootsteps.reset();
            flashingFootStepsColorHigh = false;
         }
         if (timerFlashingFootsteps.isExpired(0.1))
         {
            flashingFootStepsColorHigh = !flashingFootStepsColorHigh;
            timerFlashingFootsteps.reset();
         }
         if (getFootstepSide() == RobotSide.LEFT)
         {
            if (flashingFootStepsColorHigh)
               setColor(1.0f, 0.0f, 0.0f, 0.0f);
            else
               setColor(0.5f, 0.0f, 0.0f, 0.0f);
         }
         else
         {
            if (flashingFootStepsColorHigh)
               setColor(0.0f, 1.0f, 0.0f, 0.0f);
            else
               setColor(0.0f, 0.5f, 0.0f, 0.0f);
         }
      }
   }

   // sets color of the corresponding footstep in the list
   public void setColor(float r, float g, float b, float a)
   {
      getFootstepModelInstance().materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, r, g, b, a));
   }

   public void setFootstepModelInstance(GDXModelInstance footstepModelInstance)
   {
      this.footstepModelInstance = footstepModelInstance;
   }

   public GDXSelectablePose3DGizmo getSelectablePose3DGizmo()
   {
      return selectablePose3DGizmo;
   }

   public RobotSide getFootstepSide()
   {
      return footstepSide;
   }

   public GDXModelInstance getFootstepModelInstance()
   {
      return footstepModelInstance;
   }

   public Pose3DReadOnly getPose()
   {
      return selectablePose3DGizmo.getPoseGizmo().getPose();
   }

   public Sphere3D getBoundingSphere()
   {
      return boundingSphere;
   }

   public boolean isHovered()
   {
      return isHovered;
   }

   public double getYaw()
   {
      return tempFramePose.getYaw();
   }

   public RigidBodyTransform getFootTransformInWorld()
   {
      return getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent();
   }

   public void setFootTransform(RigidBodyTransform rigidBodyTransform)
   {
      getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent().set(rigidBodyTransform);
   }

   public void updatePose(FramePose3D footstepPose)
   {
      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      GDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
      tempFramePose.set(footstepPose);
      tempFramePose.get(getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());
      getSelectablePose3DGizmo().getPoseGizmo().updateTransforms();
   }

   /**
    * TODO: Evaluate the use of this method.
    */
   public void copyFrom(GDXImGuiBasedUI baseUI, GDXInteractableFootstep manuallyPlacedFootstep)
   {
      this.footstepIndexText = manuallyPlacedFootstep.footstepIndexText;
      this.footstepModelInstance = manuallyPlacedFootstep.footstepModelInstance;
      this.footstepSide = manuallyPlacedFootstep.getFootstepSide();
      this.selectablePose3DGizmo = manuallyPlacedFootstep.getSelectablePose3DGizmo();
      this.tempFramePose.setIncludingFrame(manuallyPlacedFootstep.tempFramePose);
      this.tempTransform.set(manuallyPlacedFootstep.tempTransform);
      this.isHovered = manuallyPlacedFootstep.isHovered;
      this.isClickedOn = manuallyPlacedFootstep.isClickedOn;
      this.textFramePose.setIncludingFrame(manuallyPlacedFootstep.textFramePose);
      this.flashingFootStepsColorHigh = manuallyPlacedFootstep.flashingFootStepsColorHigh;
   }
}
