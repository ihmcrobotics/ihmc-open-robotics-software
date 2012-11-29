package us.ihmc.graphics3DAdapter.structure;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.media.j3d.Transform3D;

import us.ihmc.graphics3DAdapter.graphics.GraphicsObject;

public class Graphics3DNode
{
   private final Transform3D transform = new Transform3D();
   private GraphicsObject graphicsObject;
   private final ArrayList<Graphics3DNode> childeren = new ArrayList<Graphics3DNode>();
   
   public Transform3D getTransform()
   {
      return transform;
   }
   public void setTransform(Transform3D transform)
   {
      this.transform.set(transform);
   }
   
   public void addChild(Graphics3DNode child)
   {
      childeren.add(child);
   }
   public List<Graphics3DNode> getChildrenNodes()
   {
      return Collections.unmodifiableList(childeren);
   }
   
   public GraphicsObject getGraphicsObject()
   {
      return graphicsObject;
   }
   
   public void setGraphicsObject(GraphicsObject graphicsObject)
   {
      this.graphicsObject = graphicsObject;
   }
}
