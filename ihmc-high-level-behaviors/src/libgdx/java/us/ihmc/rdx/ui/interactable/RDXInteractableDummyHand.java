package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.gizmo.BoxRayIntersection;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXInteractableDummyHand implements RenderableProvider
{
   private static final int NUMBER_OF_FINGERS = 2;
   private static final RigidBodyTransform[] FINGERS_TO_PALM_OPEN = new RigidBodyTransform[] {new RigidBodyTransform(), new RigidBodyTransform()};
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_OPEN[0].getRotation(), -90.0, 0.0, 0.0);
      FINGERS_TO_PALM_OPEN[0].getTranslation().set(0.11, -0.03, 0.0);

      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_OPEN[1].getRotation(), 90.0, 0.0, 180.0);
      FINGERS_TO_PALM_OPEN[1].getTranslation().set(0.11, 0.03, 0.0);
   }

   private static final RigidBodyTransform[] FINGERS_TO_PALM_NEUTRAL = new RigidBodyTransform[] {new RigidBodyTransform(), new RigidBodyTransform()};
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_NEUTRAL[0].getRotation(), 0.0, 0.0, 0.0);
      FINGERS_TO_PALM_NEUTRAL[0].getTranslation().set(0.11, -0.03, 0.0);

      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_NEUTRAL[1].getRotation(), 0.0, 0.0, 180.0);
      FINGERS_TO_PALM_NEUTRAL[1].getTranslation().set(0.11, 0.03, 0.0);
   }

   private static final RigidBodyTransform[] FINGERS_TO_PALM_CLOSE = new RigidBodyTransform[] {new RigidBodyTransform(), new RigidBodyTransform()};
   static
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_CLOSE[0].getRotation(), 15.0, 0.0, 0.0);
      FINGERS_TO_PALM_CLOSE[0].getTranslation().set(0.11, -0.03, 0.0);

      EuclidCoreMissingTools.setYawPitchRollDegrees(FINGERS_TO_PALM_CLOSE[1].getRotation(), -15.0, 0.0, 180.0);
      FINGERS_TO_PALM_CLOSE[1].getTranslation().set(0.11, 0.03, 0.0);
   }

   private final RDXInteractableFrameModel interactableHandFrameModel = new RDXInteractableFrameModel();
   private final ReferenceFrame referenceFrameHand;
   private final RDXModelInstance[] fingersModelInstances;
   private final RigidBodyTransform[] fingersTransforms;
   private final ReferenceFrame[] fingersFrames;
   private final BoxRayIntersection boxRayIntersection = new BoxRayIntersection();

   public RDXInteractableDummyHand(RDX3DPanel panel3D, RigidBodyTransform transformToParentToModify)
   {
      this.referenceFrameHand = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                       transformToParentToModify);
      ModelData handModel = RDXModelLoader.loadModelData("environmentObjects/sakeGripper/sakePalm.g3dj");
      interactableHandFrameModel.create(referenceFrameHand, transformToParentToModify, panel3D, handModel, this::calculateClosestCollision);

      ModelData fingerModel = RDXModelLoader.loadModelData("environmentObjects/sakeGripper/sakeFinger.g3dj");
      this.fingersModelInstances = new RDXModelInstance[] {new RDXModelInstance(new Model(fingerModel)), new RDXModelInstance(new Model(fingerModel))};
      this.fingersFrames = new ReferenceFrame[NUMBER_OF_FINGERS];
      this.fingersTransforms = new RigidBodyTransform[NUMBER_OF_FINGERS];
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
      {
         fingersTransforms[i] = new RigidBodyTransform(FINGERS_TO_PALM_NEUTRAL[i]);
         fingersFrames[i] = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(referenceFrameHand, fingersTransforms[i]);
      }

      panel3D.addImGui3DViewInputProcessor(this::updateFingers);
   }

   private void updateFingers(ImGui3DViewInput imGui3DViewInput)
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
      {
         fingersFrames[i].update();
         LibGDXTools.toLibGDX(fingersFrames[i].getTransformToRoot(), fingersModelInstances[i].transform);
      }
   }

   public void closeGripper()
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
         fingersTransforms[i].set(FINGERS_TO_PALM_CLOSE[i]);
   }

   public void openGripper()
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
         fingersTransforms[i].set(FINGERS_TO_PALM_OPEN[i]);
   }

   public void setGripperToNeutral()
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
         fingersTransforms[i].set(FINGERS_TO_PALM_NEUTRAL[i]);
   }

   public void setGripperClosure(double closure)
   {
      EuclidCoreMissingTools.setYawPitchRollDegrees(fingersTransforms[0].getRotation(),
                                                    closure,
                                                    fingersTransforms[0].getRotation().getPitch(),
                                                    fingersTransforms[0].getRotation().getRoll());
      EuclidCoreMissingTools.setYawPitchRollDegrees(fingersTransforms[1].getRotation(),
                                                    -closure,
                                                    fingersTransforms[1].getRotation().getPitch(),
                                                    fingersTransforms[1].getRotation().getRoll());
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      RigidBodyTransform intersectionHandOrigin = new RigidBodyTransform(interactableHandFrameModel.getReferenceFrame().getTransformToWorldFrame());
      intersectionHandOrigin.getTranslation()
                            .set(new Vector3D(interactableHandFrameModel.getReferenceFrame().getTransformToWorldFrame().getTranslationX() + 0.05,
                                              intersectionHandOrigin.getTranslationY(),
                                              intersectionHandOrigin.getTranslationZ()));

      if (boxRayIntersection.intersect(0.20, 0.1, 0.08, intersectionHandOrigin, mousePickRay))
      {
         return mousePickRay.getPoint().distance(boxRayIntersection.getFirstIntersectionToPack());
      }
      else
      {
         return Double.NaN;
      }
   }

   public float getMinGripperClosure()
   {
      return (float) Math.toDegrees(FINGERS_TO_PALM_OPEN[0].getRotation().getYaw());
   }

   public float getMaxGripperClosure()
   {
      return (float) Math.toDegrees(FINGERS_TO_PALM_CLOSE[0].getRotation().getYaw());
   }

   public float getGripperClosure()
   {
      return (float) Math.toDegrees(fingersTransforms[0].getRotation().getYaw());
   }

   public RDXPose3DGizmo getPose3DGizmo()
   {
      return interactableHandFrameModel.getPose3DGizmo();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (int i = 0; i < NUMBER_OF_FINGERS; i++)
         fingersModelInstances[i].getRenderables(renderables, pool);
   }
}

