package us.ihmc.graphics3DAdapter.structure;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.media.j3d.Transform3D;

import us.ihmc.graphics3DAdapter.graphics.LinkGraphics;

public class Graphics3DNode
{
   private final String name;
   private final Transform3D transform = new Transform3D();
   private LinkGraphics graphicsObject;
   private final ArrayList<Graphics3DNode> childeren = new ArrayList<Graphics3DNode>();
   
   public Graphics3DNode(String name)
   {
      this.name = name;
   }
   
   public synchronized Transform3D getTransform()
   {
      return transform;
   }
   public synchronized void setTransform(Transform3D transform)
   {
      this.transform.set(transform);
   }
   
   public void addChild(Graphics3DNode child)
   {
      synchronized(childeren)
      {
         childeren.add(child);
      }
   }
   public List<Graphics3DNode> getChildrenNodes()
   {
      synchronized(childeren)
      {
         return Collections.unmodifiableList(childeren);
      }
   }
   
   public LinkGraphics getGraphicsObject()
   {
      return graphicsObject;
   }
   
   public void setGraphicsObject(LinkGraphics graphicsObject)
   {
      this.graphicsObject = graphicsObject;
   }
   public String getName()
   {
      return name;
   }
}
