package us.ihmc.gdx;

import javafx.collections.ObservableList;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Transform;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;

import java.util.ArrayList;

public class GDXGraphics3DNode
{
   private final Graphics3DNode graphicsNode;
   private final GDXGraphicsObject gdxGraphicsObject;
   private final ArrayList<GDXGraphics3DNode> updatables = new ArrayList<>();

   public GDXGraphics3DNode(Graphics3DNode graphicsNode)
   {
      this(graphicsNode, null);
   }

   public GDXGraphics3DNode(Graphics3DNode graphicsNode, AppearanceDefinition appearance)
   {
      this.graphicsNode = graphicsNode;

      gdxGraphicsObject = new GDXGraphicsObject(graphicsNode.getGraphics3DObject(), appearance);
      getChildren().add(gdxGraphicsObject.getGroup());
   }

   public void update()
   {
      ObservableList<Transform> transforms = getTransforms();
      transforms.clear();
      Affine javaFxAffineTransform = new Affine();

      AffineTransform euclidAffineTransform = graphicsNode.getTransform();
      JavaFXTools.convertEuclidAffineToJavaFXAffine(euclidAffineTransform, javaFxAffineTransform);
      transforms.add(javaFxAffineTransform);

      for (int i = 0; i < updatables.size(); i++)
      {
         updatables.get(i).update();
      }
   }

   public void addChild(GDXGraphics3DNode child)
   {
      getChildren().add(child);
      updatables.add(child);
   }
}
