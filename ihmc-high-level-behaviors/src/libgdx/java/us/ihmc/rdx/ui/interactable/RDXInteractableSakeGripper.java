package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.sceneManager.RDXRenderableAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.interaction.BoxRayIntersection;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;

import java.util.Arrays;
import java.util.List;
import java.util.Set;

public class RDXInteractableSakeGripper implements RDXInteractableAffordanceTemplateHand
{
   private static final int NUMBER_OF_FINGERS = 2;
   private static final RigidBodyTransform[] FINGERS_TO_PALM_OPEN = new RigidBodyTransform[] {new RigidBodyTransform(), new RigidBodyTransform()};
   static
   {
      FINGERS_TO_PALM_OPEN[0].getTranslation().set(0.0, 0.03, 0.0);
      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_OPEN[1].getRotation(), 180.0, 0.0, 0.0);
      FINGERS_TO_PALM_OPEN[1].getTranslation().set(0.0, -0.03, 0.0);
   }

   private static final RigidBodyTransform[] FINGERS_TO_PALM_HALF_CLOSE = new RigidBodyTransform[] {new RigidBodyTransform(), new RigidBodyTransform()};
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_HALF_CLOSE[0].getRotation(), 0.0, 0.0, -65.0);
      FINGERS_TO_PALM_HALF_CLOSE[0].getTranslation().set(0.0, 0.03, 0.0);

      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_HALF_CLOSE[1].getRotation(), 180.0, 0.0, -65.0);
      FINGERS_TO_PALM_HALF_CLOSE[1].getTranslation().set(0.0, -0.03, 0.0);
   }

   private static final RigidBodyTransform[] FINGERS_TO_PALM_CLOSE = new RigidBodyTransform[] {new RigidBodyTransform(), new RigidBodyTransform()};
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_CLOSE[0].getRotation(), 0.0, 0.0, -100.0);
      FINGERS_TO_PALM_CLOSE[0].getTranslation().set(0.0, 0.03, 0.0);

      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_CLOSE[1].getRotation(), 180.0, 0.0, -100.0);
      FINGERS_TO_PALM_CLOSE[1].getTranslation().set(0.0, -0.03, 0.0);
   }

   private static final RigidBodyTransform[] FINGERS_TO_PALM_CRUSH = new RigidBodyTransform[] {new RigidBodyTransform(), new RigidBodyTransform()};
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_CRUSH[0].getRotation(), 0.0, 0.0, -106.0);
      FINGERS_TO_PALM_CRUSH[0].getTranslation().set(0.0, 0.03, 0.0);

      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_CRUSH[1].getRotation(), 180.0, 0.0, -106.0);
      FINGERS_TO_PALM_CRUSH[1].getTranslation().set(0.0, -0.03, 0.0);
   }

   private RDXInteractableFrameModel interactableHandFrameModel = new RDXInteractableFrameModel();
   private final ReferenceFrame referenceFrameHand;
   private final RDXModelInstance[] fingersModelInstances;
   private final Model[] fingersModel;
   private final RigidBodyTransform[] fingersTransforms;
   private final ReferenceFrame[] fingersFrames;
   private final BoxRayIntersection boxRayIntersection = new BoxRayIntersection();
   private SakeHandCommandOption sakeHandConfiguration;

   public RDXInteractableSakeGripper(RDX3DPanel panel3D, RigidBodyTransform transformToParentToModify, ColorDefinition color)
   {
      this.referenceFrameHand = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                       transformToParentToModify);
      ModelData handModel = RDXModelLoader.loadModelData("environmentObjects/sakeGripper/sakePalm.g3dj");
      interactableHandFrameModel.create(referenceFrameHand, transformToParentToModify, panel3D, handModel, this::calculateClosestCollision);
      interactableHandFrameModel.getModelInstance().setColor(color);

      ModelData fingerModel = RDXModelLoader.loadModelData("environmentObjects/sakeGripper/sakeFinger.g3dj");

      this.fingersModel = new Model[NUMBER_OF_FINGERS];
      this.fingersModelInstances = new RDXModelInstance[NUMBER_OF_FINGERS];
      this.fingersFrames = new ReferenceFrame[NUMBER_OF_FINGERS];
      this.fingersTransforms = new RigidBodyTransform[NUMBER_OF_FINGERS];
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
      {
         fingersModel[i] = new Model(fingerModel);
         fingersModelInstances[i] = new RDXModelInstance(fingersModel[i]);
         fingersModelInstances[i].setColor(color);
         fingersTransforms[i] = new RigidBodyTransform(FINGERS_TO_PALM_CLOSE[i]);
         fingersFrames[i] = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(referenceFrameHand, fingersTransforms[i]);
      }
      sakeHandConfiguration = SakeHandCommandOption.CLOSE;

      panel3D.getScene().addRenderableProvider(this, this::getRenderables);
      panel3D.addImGui3DViewInputProcessor(this, this::updateFingers);
   }

   private void updateFingers(ImGui3DViewInput imGui3DViewInput)
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
      {
         fingersFrames[i].update();
         LibGDXTools.toLibGDX(fingersFrames[i].getTransformToRoot(), fingersModelInstances[i].transform);
      }
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      RigidBodyTransform intersectionHandOrigin = new RigidBodyTransform(interactableHandFrameModel.getReferenceFrame().getTransformToWorldFrame());
      intersectionHandOrigin.getTranslation()
                            .set(new Vector3D(intersectionHandOrigin.getTranslationX(),
                                              intersectionHandOrigin.getTranslationY(),
                                              interactableHandFrameModel.getReferenceFrame().getTransformToWorldFrame().getTranslationZ() + 0.05));

      if (boxRayIntersection.intersect(0.08, 0.1, 0.20, intersectionHandOrigin, mousePickRay))
      {
         return mousePickRay.getPoint().distance(boxRayIntersection.getFirstIntersectionToPack());
      }
      else
      {
         return Double.NaN;
      }
   }

   @Override
   public void setToConfiguration(String configuration)
   {
      switch (SakeHandCommandOption.valueOf(configuration))
      {
         case FULLY_OPEN -> fullyOpenGripper();
         case OPEN -> openGripper();
         case CLOSE -> closeGripper();
         case GRIP_HARD -> crushGripper();
      }
   }

   @Override
   public List<String> getAvailableConfigurations()
   {
      return Arrays.asList("FULLY_OPEN", "OPEN", "CLOSE", "GRIP_HARD");
   }

   @Override
   public void setToDefaultConfiguration()
   {
      closeGripper();
   }

   @Override
   public String getConfiguration()
   {
      return sakeHandConfiguration.toString();
   }

   @Override
   public ReferenceFrame getReferenceFrameHand()
   {
      return referenceFrameHand;
   }

   @Override
   public boolean isSelected()
   {
      return interactableHandFrameModel.isSelected();
   }

   @Override
   public void setSelected(boolean selected)
   {
      interactableHandFrameModel.setSelected(selected);
   }

   @Override
   public RDXPose3DGizmo getPose3DGizmo()
   {
      return interactableHandFrameModel.getPoseGizmo();
   }

   @Override
   public boolean hasGripper()
   {
      return true;
   }

   private void crushGripper()
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
         fingersTransforms[i].set(FINGERS_TO_PALM_CRUSH[i]);
      sakeHandConfiguration = SakeHandCommandOption.GRIP_HARD;
   }

   private void closeGripper()
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
         fingersTransforms[i].set(FINGERS_TO_PALM_CLOSE[i]);
      sakeHandConfiguration = SakeHandCommandOption.CLOSE;
   }

   private void fullyOpenGripper()
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
         fingersTransforms[i].set(FINGERS_TO_PALM_OPEN[i]);
      sakeHandConfiguration = SakeHandCommandOption.FULLY_OPEN;
   }

   private void openGripper()
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
         fingersTransforms[i].set(FINGERS_TO_PALM_HALF_CLOSE[i]);
      sakeHandConfiguration = SakeHandCommandOption.OPEN;
   }

   @Override
   public float getMinGripperClosure()
   {
      return (float) Math.toDegrees(FINGERS_TO_PALM_OPEN[0].getRotation().getRoll());
   }

   @Override
   public float getMaxGripperClosure()
   {
      return (float) Math.toDegrees(FINGERS_TO_PALM_CRUSH[0].getRotation().getRoll());
   }

   @Override
   public void setGripperClosure(double closure)
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(fingersTransforms[0].getRotation(),
                                                    fingersTransforms[0].getRotation().getYaw(),
                                                    fingersTransforms[0].getRotation().getPitch(),
                                                    closure);
      EuclidCoreMissingTools.setYawPitchRollDegrees(fingersTransforms[1].getRotation(),
                                                    180 + fingersTransforms[0].getRotation().getYaw(),
                                                    fingersTransforms[1].getRotation().getPitch(),
                                                    closure);
   }

   @Override
   public float getGripperClosure()
   {
      return (float) Math.toDegrees(fingersTransforms[0].getRotation().getRoll());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (interactableHandFrameModel.isShowing())
         for (int i = 0; i < NUMBER_OF_FINGERS; i++)
            fingersModelInstances[i].getRenderables(renderables, pool);
   }

   @Override
   public void removeRenderables(RDX3DPanel panel3D)
   {
      interactableHandFrameModel.destroy(panel3D);
      panel3D.getScene().removeRenderable(this);
      panel3D.removeImGui3DViewInputProcessor(this);
   }
}