package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.mesh.RDXMutableMultiLineModel;

import java.util.List;

/**
 * A white line for the translation part and little coordinate frame graphics to show orientation change.
 * Orientation component is only shown if the orientation change is significant.
 */
public class RDXTrajectoryGraphic
{
   public static final float OPACITY = 0.7f;
   private final RecyclingArrayList<Point3D> positions = new RecyclingArrayList<>(Point3D::new);
   private final RDXMutableMultiLineModel positionTrajectoryGraphic = new RDXMutableMultiLineModel();

   private final RecyclingArrayList<Pose3D> poses = new RecyclingArrayList<>(Pose3D::new);
   private RecyclingArrayList<RDXReferenceFrameGraphic> referenceFrameGraphics;

   private final RotationMatrix identity = new RotationMatrix();
   private final RotationMatrix relativeRotation = new RotationMatrix();
   private final RotationMatrix endOrientation = new RotationMatrix();

   /** Straight line trajectory. */
   public void update(double lineWidth, RigidBodyTransformReadOnly start, RigidBodyTransformReadOnly end)
   {
      positions.clear();
      poses.clear();
      positions.add().set(start.getTranslation());
      positions.add().set(end.getTranslation());
      poses.add().set(start);
      poses.add().set(end);

      updateInternal(lineWidth);
   }

   public void update(double lineWidth, List<? extends RigidBodyTransformReadOnly> posesReadOnly)
   {
      positions.clear();
      poses.clear();
      for (RigidBodyTransformReadOnly pose : posesReadOnly)
      {
         positions.add().set(pose.getTranslation());
         poses.add().set(pose);
      }

      updateInternal(lineWidth);
   }

   private void updateInternal(double lineWidth)
   {
      positionTrajectoryGraphic.update(positions, lineWidth, Color.WHITE);
      positionTrajectoryGraphic.accessModelIfExists(modelInstance -> modelInstance.setOpacity(OPACITY));

      if (referenceFrameGraphics == null)
      {
         referenceFrameGraphics = new RecyclingArrayList<>(() -> new RDXReferenceFrameGraphic(0.03));
      }
      referenceFrameGraphics.clear();

      // Draw frames only at a regular distance interval
      double spaceBetweenDraws = 0.02;
      double remainingDistanceBeforeDraw = spaceBetweenDraws;

      for (int i = 1; i < poses.size(); i++)
      {
         Pose3D start = poses.get(i - 1);
         Pose3D end = poses.get(i);

         double segmentLength = start.getTranslation().distance(end.getTranslation());
         double remainingSegmentLength = segmentLength;
         double segmentTraversal = 0.0;

         while (remainingSegmentLength > remainingDistanceBeforeDraw)
         {
            segmentTraversal += remainingDistanceBeforeDraw;

            RDXReferenceFrameGraphic referenceFrameGraphic = referenceFrameGraphics.add();
            referenceFrameGraphic.getFramePose3D().interpolate(start, end, segmentTraversal / segmentLength);
            referenceFrameGraphic.updateFromFramePose();
            referenceFrameGraphic.setOpacity(OPACITY);

            // Don't show frames if there is not much orientation change
            if (referenceFrameGraphics.size() > 1)
            {
               endOrientation.set(referenceFrameGraphic.getFramePose3D().getRotation());
               relativeRotation.set(referenceFrameGraphics.get(referenceFrameGraphics.size() - 2).getFramePose3D().getRotation());
               relativeRotation.invert();
               relativeRotation.multiply(endOrientation);

               double distance = relativeRotation.distance(identity);
               if (distance < 0.075)
               {
                  referenceFrameGraphics.remove(referenceFrameGraphics.size() - 1);
               }
            }

            remainingSegmentLength -= spaceBetweenDraws;
            remainingDistanceBeforeDraw = spaceBetweenDraws;
         }

         remainingDistanceBeforeDraw -= remainingSegmentLength;
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      positionTrajectoryGraphic.getRenderables(renderables, pool);

      if (referenceFrameGraphics != null)
      {
         for (RDXReferenceFrameGraphic referenceFrameGraphic : referenceFrameGraphics)
         {
            referenceFrameGraphic.getRenderables(renderables, pool);
         }
      }
   }
}