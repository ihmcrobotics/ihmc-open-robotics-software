package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;

import java.util.ArrayList;

public class GDXGraphics3DNode
{
   private final Graphics3DNode graphicsNode;
   private final GDXGraphicsObject gdxGraphicsObject;

   private final ArrayList<GDXGraphics3DNode> children = new ArrayList<>();

   private final AffineTransform tempTransform = new AffineTransform();

   public GDXGraphics3DNode(Graphics3DNode graphicsNode)
   {
      this(graphicsNode, null);
   }

   public GDXGraphics3DNode(Graphics3DNode graphicsNode, AppearanceDefinition appearance)
   {
      this.graphicsNode = graphicsNode;

      gdxGraphicsObject = new GDXGraphicsObject(graphicsNode.getGraphics3DObject(), appearance);
   }

   public void update()
   {
      tempTransform.setIdentity();
      updateInternal(tempTransform);
   }

   private void updateInternal(AffineTransform parentTransform)
   {
      tempTransform.set(parentTransform);
      tempTransform.multiply(graphicsNode.getTransform());
      gdxGraphicsObject.setWorldTransform(tempTransform);

      for (GDXGraphics3DNode child : children)
      {
         child.updateInternal(tempTransform);
      }
   }

   public void addChild(GDXGraphics3DNode child)
   {
      children.add(child);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXGraphics3DNode child : children)
      {
         child.getRenderables(renderables, pool);
      }

      gdxGraphicsObject.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      for (GDXGraphics3DNode child : children)
      {
         child.destroy();
      }

      gdxGraphicsObject.destroy();
   }
}
