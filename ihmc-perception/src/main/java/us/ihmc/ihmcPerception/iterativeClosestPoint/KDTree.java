package us.ihmc.ihmcPerception.iterativeClosestPoint;

import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;
import java.util.Random;

public class KDTree {

    public ArrayList<KDNode> pointList;

    int size = 0;
    KDNode root;

    public KDTree(){
        pointList = new ArrayList<>();
    }

    public boolean insert(KDNode node){
        size += 1;
        pointList.add(node);
        if (root == null) {
            root = node;
            return true;
        }
        else {
            return insert(root, node);
        }
    }

    private boolean insert(KDNode parent, KDNode node){

        node.level += 1;


        int comp = node.compareTo(parent);
        // System.out.println(node + " " + parent + " Comp:" + comp);


        if (comp == -1){
            // System.out.println("Going Left: " + parent.level);
            if(parent.left == null) parent.left = node;
            else return insert(parent.left, node);
        }else if(comp == 1){
            // System.out.println("Going Right: " + parent.level);
            if(parent.right == null) parent.right = node;
            else return insert(parent.right, node);
        }else{
            return false;
        }
        return true;
    }

    public void search(KDNode node){
        if(node.right != null){
            System.out.println("Going Right");
            search(node.right);
        }
        System.out.println(node);
        if(node.left != null){
            System.out.println("Going Left");
            search(node.left);
        }
    }

    public static void insertPoints(KDTree tree, ArrayList<Point3D> points){
        for(int i = 0; i<points.size(); i++){
            tree.insert(new KDNode(points.get(i), 0, i));
        }
    }

    public static void insertRandomBall(KDTree tree, Point3D center, ArrayList<Point3D> points, int num, double scale){
        for(int i = 0; i<num; i++){
            Random rand = new Random();
            Point3D randomPoint = new Point3D(rand.nextGaussian()*scale, rand.nextGaussian()*scale, rand.nextGaussian()*scale);
            System.out.println(randomPoint);
            randomPoint.add(center);
            if (points != null) points.add(randomPoint);
            else tree.insert(tree.root, new KDNode(randomPoint.getX(), randomPoint.getY(), randomPoint.getZ(), 0, i));
        }
    }

    public KDNode nearestNeighbors(KDNode node){
        return nearestNeighbors(node, root);
    }

    private KDNode nearestNeighbors(KDNode node, KDNode current){
        if (current == null) return null;

        System.out.println(node + " " + current);

        KDNode best;
        int comp = node.compareTo(current);

        if(comp==0) return current;
        else best = nearestNeighbors(node, (comp==1 ? current.right : current.left));

        best = closerNode(node, current, best);
        if(current.perpendicularDistance(node) < node.distTo(best)){
            best = closerNode(node, nearestNeighbors(node, (comp==1 ? current.right : current.left)), best);
        }
        return best;
    }

    public KDNode closerNode(KDNode pt, KDNode p1, KDNode p2) {
        if(p1 == null) return p2;
        if(p2 == null) return p1;

        return pt.distTo(p1) < pt.distTo(p2)? p1 : p2;
    }

}
