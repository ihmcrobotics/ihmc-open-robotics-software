package us.ihmc.gdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.math.Matrix4;

public class GDXModelInstanceScaler
{
   private final Node originalNode;
   private final ModelInstance modelInstance;
   private Matrix4 scaleOnlyTransform;
   private Matrix4 poseTransform;

   public GDXModelInstanceScaler(ModelInstance modelInstance)
   {
      originalNode = modelInstance.nodes.removeIndex(0);
      this.modelInstance = modelInstance;
      scaleOnlyTransform = originalNode.localTransform;
      Node posedNode = new Node();
      poseTransform = posedNode.localTransform;
      posedNode.addChild(originalNode);
      modelInstance.nodes.add(posedNode);
   }

   public void scale(double scaleFactor)
   {
      float scaleFactorFloat = (float) scaleFactor;
      scaleOnlyTransform.scale(scaleFactorFloat, scaleFactorFloat, scaleFactorFloat);
      originalNode.calculateWorldTransform();
   }

   public Matrix4 getPoseTransform()
   {
      return modelInstance.transform;
   }
}
