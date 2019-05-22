package us.ihmc.javaFXVisualizers;

import javafx.scene.Node;
import javafx.scene.transform.Affine;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.javaFXToolkit.JavaFXTools;

public class JavaFXGraphicTools
{
   public static void setNodeTransformFromPose(Node node, FramePose3DBasics pose)
   {
      node.getTransforms().clear();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.set(pose.getOrientation(), pose.getPosition());
      Affine affine = JavaFXTools.convertRigidBodyTransformToAffine(rigidBodyTransform);
      node.getTransforms().add(affine);
   }

   public static void setNodeTransformFromPose(Node node, FramePose3DBasics pose, double scale)
   {
      node.getTransforms().clear();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.set(pose.getOrientation(), pose.getPosition());
      Affine affine = JavaFXTools.convertRigidBodyTransformToAffine(rigidBodyTransform);
      affine.appendScale(scale, scale);
      node.getTransforms().add(affine);
   }

   public static void setNodePosition(Node node, Tuple3DReadOnly position)
   {
      node.setTranslateX(position.getX());
      node.setTranslateY(position.getY());
      node.setTranslateZ(position.getZ());
   }
}