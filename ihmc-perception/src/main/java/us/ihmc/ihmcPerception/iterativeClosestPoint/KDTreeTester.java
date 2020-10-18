package us.ihmc.ihmcPerception.iterativeClosestPoint;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.random.RandomGeometry;

import java.util.Random;

public class KDTreeTester
{

   public KDTreeTester(KDTree tree)
   {

      // tree.insert(tree.root, new KDNode(0.1f,0.2f,0.3f, 0));
      // tree.insert(tree.root, new KDNode(-0.2f,0.1f,-0.3f, 0));
      // tree.insert(tree.root, new KDNode(0.3f,-0.1f,0.2f, 0));
      // tree.insert(tree.root, new KDNode(-0.1f,0.2f,-0.3f, 0));
      // tree.insert(tree.root, new KDNode(0.4f, -0.24f, 0.3, 0));
      // tree.insert(tree.root, new KDNode(0.1f, 0.1f, 0.1, 0));

      Point3D first = new Point3D(1, 1, 1);
      Point3D second = new Point3D(1, -1, 1);
      Point3D third = new Point3D(-1, 1, -1);

      KDTree.insertRandomBall(tree, first, null, 100, 0.1f);
      KDTree.insertRandomBall(tree, second, null, 100, 0.1f);
      KDTree.insertRandomBall(tree, third, null, 100, 0.1f);
   }

   public static void main(String[] args){
      KDTree tree = new KDTree();
      new KDTreeTester(tree);
   }

}
