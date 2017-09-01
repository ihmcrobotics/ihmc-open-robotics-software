package us.ihmc.javaFXToolkit.node;

import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.transform.Affine;
import javafx.scene.transform.Transform;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.JavaFXTools;

public class JavaFXGraphics3DNode extends Group
{
   private final Graphics3DNode graphicsNode;
   private final JavaFXGraphicsObject javaFXGraphicsObject;

   public JavaFXGraphics3DNode(Graphics3DNode graphicsNode)
   {
      this.graphicsNode = graphicsNode;

      javaFXGraphicsObject = new JavaFXGraphicsObject(graphicsNode.getGraphics3DObject());
      this.getChildren().add(javaFXGraphicsObject.getGroup());
   }

   public void update()
   {
      ObservableList<Transform> transforms = this.getTransforms();
      transforms.clear();
      Affine javaFxAffineTransform = new Affine();

      AffineTransform euclidAffineTransform = graphicsNode.getTransform();
      JavaFXTools.convertEuclidAffineToJavaFXAffine(euclidAffineTransform, javaFxAffineTransform);
      transforms.add(javaFxAffineTransform);
   }

   public void addChild(Node child)
   {
      this.getChildren().add(child);
   }
}
