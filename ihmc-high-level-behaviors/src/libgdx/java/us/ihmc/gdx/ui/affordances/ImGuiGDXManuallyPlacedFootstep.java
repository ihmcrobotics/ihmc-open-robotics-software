package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.gdx.GDX3DSituatedText;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDXRenderableAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.StepCheckIsPointInsideAlgorithm;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;

import java.util.ArrayList;
import java.util.function.Function;

public class ImGuiGDXManuallyPlacedFootstep
{
   private final GDX3DSituatedText footstepIndexText;
   private final GDXRenderableAdapter renderableAdapter;
   private GDXModelInstance footstepModelInstance;
   private RobotSide footstepSide;

   private GDXSelectablePose3DGizmo selectablePose3DGizmo;
   private FramePose3D tempFramePose = new FramePose3D();
   private RigidBodyTransform tempTransform = new RigidBodyTransform();
   public boolean pickSelected;
   GDXImGuiBasedUI baseUI;

   public Sphere3D boundingSphere = new Sphere3D(0.1);
   private boolean isClickedOn;
   private int index;
   private final FramePose3D textFramePose = new FramePose3D();
   private final ArrayList<GDX3DSituatedText> textRenderables = new ArrayList<>();

   private volatile Runnable buildMeshAndCreateModelInstance = null;
   //public GDXSelectablePose3DGizmo gizmo = new GDXSelectablePose3DGizmo();
   Timer timerFlashingFootsteps = new Timer();
   boolean flashingFootStepsColorHigh = false;


   public ImGuiGDXManuallyPlacedFootstep(GDXImGuiBasedUI baseUI, RobotSide footstepSide, int index)
   {
      this.footstepSide = footstepSide;
      this.baseUI = baseUI;
      this.index = index;

      if (footstepSide.equals(RobotSide.LEFT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_left.g3dj"));
      }
      else if (footstepSide.equals(RobotSide.RIGHT))
      {
         footstepModelInstance = new GDXModelInstance(GDXModelLoader.load("models/footsteps/footstep_right.g3dj"));
      }

      baseUI.getPrimaryScene().addModelInstance(footstepModelInstance, GDXSceneLevel.VIRTUAL);

      selectablePose3DGizmo = new GDXSelectablePose3DGizmo();

      selectablePose3DGizmo.create(baseUI.getPrimary3DPanel().getCamera3D());

      footstepIndexText = new GDX3DSituatedText("" + footstepSide.getSideNameFirstLetter() + index);

      renderableAdapter = new GDXRenderableAdapter(footstepModelInstance::getRenderables, GDXSceneLevel.VIRTUAL);

      textRenderables.add(footstepIndexText);

   }

   public void setGizmoPose(double x, double y, double z)
   {
      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      tempFramePose.getPosition().set(x, y, z);
      tempFramePose.get(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
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

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      StepCheckIsPointInsideAlgorithm stepCheckIsPointInsideAlgorithm = new StepCheckIsPointInsideAlgorithm();
      stepCheckIsPointInsideAlgorithm.setup(boundingSphere.getRadius(), boundingSphere.getPosition());

      Function<Point3DReadOnly, Boolean> isPointInside = boundingSphere::isPointInside;
      pickSelected = !Double.isNaN(stepCheckIsPointInsideAlgorithm.intersect(input.getPickRayInWorld(), 100, isPointInside));
      isClickedOn = pickSelected && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left);


      // TODO: mouse hovering on the footstep. (get foot validity warning text when this happens)
      if(pickSelected)
      {
         if(footstepSide == RobotSide.LEFT)
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 1.0f,0.0f,0.0f, 0.0f));
         else
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.0f,1.0f,0.0f, 0.0f));
      }
      else
      {
         if(footstepSide == RobotSide.LEFT)
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.5f,0.0f,0.0f, 0.0f));
         else
            footstepModelInstance.materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, 0.0f,0.5f,0.0f, 0.0f));
      }

      selectablePose3DGizmo.process3DViewInput(input, pickSelected);

      footstepModelInstance.transform.setToRotationRad(selectablePose3DGizmo.getPoseGizmo().getPose().getOrientation().getX32(),
                                                       selectablePose3DGizmo.getPoseGizmo().getPose().getOrientation().getY32(),
                                                       selectablePose3DGizmo.getPoseGizmo().getPose().getOrientation().getZ32(),
                                                       (float) selectablePose3DGizmo.getPoseGizmo().getPose().getOrientation().angle());
      footstepModelInstance.transform.setTranslation(selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getX32(),
                                                     selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getY32(),
                                                     selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getZ32());
      boundingSphere.getPosition().set(selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getX32(),
                                       selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getY32(),
                                       selectablePose3DGizmo.getPoseGizmo().getPose().getPosition().getZ32());


      /*
      if (unmodifiedButHovered)
      {
         if (hasMultipleFrames && controlToGraphicTransform == null) // we just need to do this once
         {
            controlToGraphicTransform = new RigidBodyTransform();
            tempFramePose.setToZero(graphicFrame);
            tempFramePose.changeFrame(controlFrame);
            tempFramePose.get(controlToGraphicTransform);
            controlToCollisionTransform = new RigidBodyTransform();
            tempFramePose.setToZero(collisionFrame);
            tempFramePose.changeFrame(controlFrame);
            tempFramePose.get(controlToCollisionTransform);
         }

         if (hasMultipleFrames)
         {
            highlightModel.setPose(controlFrame.getTransformToWorldFrame(), controlToGraphicTransform);
         }
         else
         {
            highlightModel.setPose(controlFrame.getTransformToWorldFrame());
         }
      }

      if (becomesModified)
      {
         modified = true;
         collisionLink.setOverrideTransform(true);
         selectablePose3DGizmo.getPoseGizmo().getTransformToParent().set(controlFrame.getTransformToWorldFrame());
      }

      if (modifiedButNotSelectedHovered)
      {
         highlightModel.setTransparency(0.7);
      }
      else
      {
         highlightModel.setTransparency(0.5);
      }

      if (modified)
      {
         if (hasMultipleFrames)
         {
            tempTransform.set(controlToCollisionTransform);
            selectablePose3DGizmo.getPoseGizmo().getTransformToParent().transform(tempTransform);
            collisionLink.setOverrideTransform(true).set(tempTransform);
            highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent(), controlToGraphicTransform);
         }
         else
         {
            collisionLink.setOverrideTransform(true).set(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
            highlightModel.setPose(selectablePose3DGizmo.getPoseGizmo().getTransformToParent());
         }
      }

      if (selectablePose3DGizmo.isSelected() && executeMotionKeyPressed)
      {
         onSpacePressed.run();
      }*/
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      selectablePose3DGizmo.getVirtualRenderables(renderables, pool);

      for (GDX3DSituatedText textRenderable : textRenderables)
      {
         textRenderable.getRenderables(renderables, pool);
      }
   }

   public void update()
   {


      selectablePose3DGizmo.getPoseGizmo().getPose().get(tempTransform);
      double textHeight = 0.08;
      textFramePose.setToZero(selectablePose3DGizmo.getPoseGizmo().getPose().getReferenceFrame());
      textFramePose.set(selectablePose3DGizmo.getPoseGizmo().getPose());

      textFramePose.appendYawRotation(-Math.PI / 2.0);
      textFramePose.appendTranslation(-0.03,0.0,0.035); //Make text higher in z direction, so it's not inside the foot
      textFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      GDXTools.toGDX(textFramePose, tempTransform, footstepIndexText.getModelInstance().transform);
      footstepIndexText.scale((float) textHeight);


   }


   public Pose3DReadOnly getPose()
   {
      return selectablePose3DGizmo.getPoseGizmo().getPose();
   }

   public boolean isClickedOn()
   {
      return isClickedOn;
   }

   public Sphere3D getBoundingSphere()
   {
      return boundingSphere;
   }

   public GDXRenderableAdapter getRenderableAdapter()
   {
      return renderableAdapter;
   }

   public boolean isPickSelected()
   {
      return pickSelected;
   }

   public double getYaw()
   {
      return tempFramePose.getYaw();
   }

   // sets color of the corresponding footstep in the list
   public void setColor(float r, float g, float b, float a)
   {
      getFootstepModelInstance().materials.get(0).set(new ColorAttribute(ColorAttribute.Diffuse, r, g, b, a));
   }

   public void flashFootstepsWhenBadPlacement(BipedalFootstepPlannerNodeRejectionReason reason, ImGuiGDXManuallyPlacedFootstepChecker stepChecker)
   {
      if(reason == null)
      {
         if(getFootstepSide() == RobotSide.LEFT)
         {
            if(isPickSelected())
               setColor(1.0f,0.0f,0.0f,0.0f);
            else
               setColor(0.5f,0.0f,0.0f,0.0f);
         }
         else
         {
            if(isPickSelected())
               setColor(0.0f,1.0f,0.0f,0.0f);
            else
               setColor(0.0f,0.5f,0.0f,0.0f);
         }
      }
      else
      {
         if(!timerFlashingFootsteps.hasBeenSet())
         {
            timerFlashingFootsteps.reset();
            flashingFootStepsColorHigh = false;
         }
         if(timerFlashingFootsteps.isExpired(0.1))
         {
            flashingFootStepsColorHigh = !flashingFootStepsColorHigh;
            timerFlashingFootsteps.reset();
         }
         if(getFootstepSide() == RobotSide.LEFT)
         {
            if(flashingFootStepsColorHigh)
               setColor(1.0f,0.0f,0.0f,0.0f);
            else
               setColor(0.5f,0.0f,0.0f,0.0f);
         }
         else
         {
            if(flashingFootStepsColorHigh)
               setColor(0.0f,1.0f,0.0f,0.0f);
            else
               setColor(0.0f,0.5f,0.0f,0.0f);
         }
      }
   }
}
