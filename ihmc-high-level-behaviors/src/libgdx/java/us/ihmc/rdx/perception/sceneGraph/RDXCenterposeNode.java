package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.RDX3DSituatedTextData;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;

import java.util.Set;

public class RDXCenterposeNode extends RDXDetectableSceneNode
{
   private final CenterposeNode centerposeNode;
   private Set<RDXSceneLevel> sceneLevels = Set.of(RDXSceneLevel.MODEL);
   private final RDX3DSituatedText text;
   private final FramePose3D textPose = new FramePose3D();
   private RDX3DSituatedTextData previousTextData;
   private ModelInstance markerModelInstance;
   private FramePoint3D[] vertices3D = new FramePoint3D[8];
   private Point3D[] vertices = new Point3D[8];
   private final RDX3DPanel panel3D;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();

   public RDXCenterposeNode(CenterposeNode centerposeNode, RDX3DPanel panel3D)
   {
      super(centerposeNode);
      this.panel3D = panel3D;
      this.centerposeNode = centerposeNode;
      this.text = new RDX3DSituatedText("test", 0.05f);
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      super.update(modificationQueue);

      vertices = centerposeNode.getVertices3D();
      for (int i = 0; i < vertices.length; i++)
      {
         if (vertices3D[i] == null)
         {
            vertices3D[i] = new FramePoint3D();
         }
         vertices3D[i].changeFrame(ReferenceFrame.getWorldFrame());
         vertices3D[i].interpolate(vertices[i], 0.2);
      }
      Model markerModel = RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices3D, 0.005, Color.WHITE));

      if (markerModelInstance != null)
      {
         markerModelInstance.model.dispose();
      }
      markerModelInstance = new RDXModelInstance(markerModel);

      String objectType = centerposeNode.getObject_type();
      double confidence = centerposeNode.getConfidence();

      if (previousTextData != null)
         previousTextData.dispose();
      previousTextData = text.setTextWithoutCache(objectType + " %.1f".formatted(confidence));

      textPose.setToZero(panel3D.getCamera3D().getCameraFrame());
      textPose.getOrientation().appendPitchRotation(3.0 / 2.0 * Math.PI);
      textPose.getOrientation().appendYawRotation(-Math.PI / 2.0);

      textPose.changeFrame(ReferenceFrame.getWorldFrame());
      textPose.getPosition().set(centerposeNode.getNodeFrame().getTransformToWorldFrame().getTranslation());  // Not sure if this is correct?

      LibGDXTools.toLibGDX(textPose, tempTransform, text.getModelTransform());
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);
      ImGui.text("Marker ID: %d".formatted(centerposeNode.getMarkerID()));
      ImGui.sameLine();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);
      if (sceneLevelCheck(sceneLevels))
      {
         if (markerModelInstance != null)
         {
            text.getRenderables(renderables, pool);
            markerModelInstance.getRenderables(renderables, pool);
         }
      }
   }

   public boolean sceneLevelCheck(Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXSceneLevel sceneLevel : this.sceneLevels)
         if (sceneLevels.contains(sceneLevel))
            return true;
      return false;
   }
}
