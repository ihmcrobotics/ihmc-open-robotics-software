package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;

import java.util.ArrayList;

public class GDXGraphics3DNode
{

   private final Graphics3DNode graphicsNode;
   private final GDXGraphicsObject gdxGraphicsObject;
   private final ArrayList<GDXGraphics3DNode> updatables = new ArrayList<>();

   private final ArrayList<GDXGraphics3DNode> children = new ArrayList<>();

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
      gdxGraphicsObject.setTransform(graphicsNode.getTransform());

      for (GDXGraphics3DNode child : children)
      {
         child.update();
      }
   }

   public void addChild(GDXGraphics3DNode child)
   {
      children.add(child);
      addChild(child);
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
