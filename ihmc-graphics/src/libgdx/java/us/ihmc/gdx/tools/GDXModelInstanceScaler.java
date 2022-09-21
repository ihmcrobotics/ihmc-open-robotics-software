package us.ihmc.gdx.tools;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.model.Node;
import com.badlogic.gdx.math.Matrix4;

import java.util.ArrayList;

public class GDXModelInstanceScaler
{
   private final ModelInstance modelInstance;
   private record OriginalNode(Node originalNode, Matrix4 originalLocalTransform) { }
   private final ArrayList<OriginalNode> originalNodes = new ArrayList<>();

   public GDXModelInstanceScaler(ModelInstance modelInstance)
   {
      this.modelInstance = modelInstance;

      ArrayList<Node> additionalNodes = new ArrayList<>();

      for (int i = 0; i < modelInstance.nodes.size; i++)
      {
         Node originalNode = modelInstance.nodes.get(i);
         Node additionalNodeInChainForTransformToWorld = new Node();
         additionalNodeInChainForTransformToWorld.localTransform.set(originalNode.localTransform);
         additionalNodeInChainForTransformToWorld.addChild(originalNode);
         originalNodes.add(new OriginalNode(originalNode, new Matrix4(originalNode.localTransform)));
         additionalNodes.add(additionalNodeInChainForTransformToWorld);
      }

      modelInstance.nodes.clear();
      for (Node additionalNode : additionalNodes)
      {
         modelInstance.nodes.add(additionalNode);
      }

      scale(1.0);
   }

   public void scale(double scaleFactor)
   {
      float scaleFactorFloat = (float) scaleFactor;

      for (OriginalNode originalNode : originalNodes)
      {
         originalNode.originalNode().localTransform.set(originalNode.originalLocalTransform());
         originalNode.originalNode().localTransform.scale(scaleFactorFloat, scaleFactorFloat, scaleFactorFloat);
         originalNode.originalNode().calculateWorldTransform();
      }
   }

   public Matrix4 getPoseTransform()
   {
      return modelInstance.transform;
   }
}
