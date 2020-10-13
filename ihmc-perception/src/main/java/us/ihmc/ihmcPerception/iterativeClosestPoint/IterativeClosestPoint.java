package us.ihmc.ihmcPerception.iterativeClosestPoint;

import org.ejml.data.DGrowArray;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.SingularOps_DDRM;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import java.awt.*;
import java.io.File;
import java.io.FileNotFoundException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;

public class IterativeClosestPoint {

    private Point3D centerPoints3D;
    private Point3D centerPoints3DTransformed;
    private RotationMatrix rotation;
    private Vector3D translation;

    private double threshold = 0.4f;

    private String dataPath = "./ihmc-open-robotics-software/ihmc-perception/src/main/java/us/ihmc/ihmcPerception/lineSegmentDetector/";

    public void loadData(String pcdFile, ArrayList<Point3D> pcl, ArrayList<Color> colors) {
        String path = Paths.get(dataPath + pcdFile).toAbsolutePath().normalize().toString();
        // System.out.println(path);

        File myObj = new File(path);

        Scanner myReader = null;
        try {
            myReader = new Scanner(myObj);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        int k = 0;
        boolean dataStarts = false;
        assert myReader != null;
        while (myReader.hasNextLine()) {
            String data = myReader.nextLine();
            if (dataStarts) {
                double x, y, z;
                double[] pcdPoint = Arrays.stream(data.split("\\s+"))
                        .mapToDouble(Double::parseDouble)
                        .toArray();

                if (k % 128 == 0) {
                    pcl.add(new Point3D((float) pcdPoint[0] * 1f, (float) pcdPoint[1] * 1f, (float) pcdPoint[2] * 1f));
                    colors.add(new Color((int) pcdPoint[3], (int) pcdPoint[4], (int) pcdPoint[5]));
                }
                // System.out.println(Arrays.toString(pcdPoint));
                k++;
            }
            if (data.contains("DATA")) {
                dataStarts = true;
            }
        }
        myReader.close();

        System.out.println("Total Points:" + k);
    }

    public double alignPointClouds(Point3D[] points3D, Point3D[] points3DTransformed, RotationMatrix rotationToPack, Vector3D translationToPack) {

        double error = 0;

        centerPointCloud(points3DTransformed);
        int[] corresp = new int[points3D.length];
        findCorrespondences(points3D, points3DTransformed, corresp);

        DMatrixRMaj crossCovariance = new DMatrixRMaj(3, 3);
        error = computeCrossCovariance(points3D, points3DTransformed, corresp, crossCovariance);

        getRotation(crossCovariance, rotationToPack);
        transformPointCloud(points3DTransformed, rotationToPack, null);

        return error;
    }

    public double alignPointTrees(KDTree kdt1, KDTree kdtTransformed, RotationMatrix rotationToPack, Vector3D translationToPack) {

        double error = 0;

        Point3D[] points3D = (Point3D[]) kdt1.pointList.toArray();
        Point3D[] points3DTransformed = (Point3D[]) kdtTransformed.pointList.toArray();

        centerPointCloud(points3DTransformed);

        DMatrixRMaj crossCovariance = new DMatrixRMaj(3, 3);
        error = computeTreeCrossCovariance(kdt1, kdtTransformed, crossCovariance);

        getRotation(crossCovariance, rotationToPack);
        transformPointCloud(points3DTransformed, rotationToPack, null);

        return error;
    }

    public void findCorrespondences(Point3D[] pcl1, Point3D[] pcl2, int[] corresp) {
        for (int i = 0; i < pcl1.length; i++) {
            double minDist = Double.MAX_VALUE;
            for (int j = 0; j < pcl2.length; j++) {
                Vector3D pq = new Vector3D();
                pq.sub(pcl1[i], pcl2[j]);
                double dist = pq.length();

                if (dist < minDist) {
                    minDist = dist;
                    corresp[i] = j;
                }
            }
        }
    }

    public double computeTreeCrossCovariance(KDTree kdt1, KDTree kdt2, DMatrixRMaj matrixToPack) {
        double total = 0;
        for (int i = 0; i < kdt1.size; i++) {

            Point3D p1 = kdt1.pointList.get(i).point3d;
            Point3D p2 = kdt2.nearestNeighbors(kdt1.pointList.get(i)).point3d;

            Vector3D correspVec = new Vector3D();
            correspVec.sub(p1, p2);
            double dist = correspVec.length();

            // System.out.println(dist);

            total += dist;
            if (dist < threshold) {
                DMatrixRMaj p = new DMatrixRMaj(1, 3);
                DMatrixRMaj q = new DMatrixRMaj(1, 3);

                p.set(0, 0, p1.getX());
                p.set(0, 1, p1.getY());
                p.set(0, 2, p1.getZ());

                q.set(0, 0, p2.getX());
                q.set(0, 1, p2.getY());
                q.set(0, 2, p2.getZ());

                CommonOps_DDRM.transpose(p);
                DMatrixRMaj pq = new DMatrixRMaj(3, 3);
                CommonOps_DDRM.mult(p, q, pq);
                CommonOps_DDRM.add(matrixToPack, pq, matrixToPack);
            }
        }
        return total;
    }

    public double computeCrossCovariance(Point3D[] pcl1, Point3D[] pcl2, int[] corresp, DMatrixRMaj matrixToPack) {
        double total = 0;
        for (int i = 0; i < pcl1.length; i++) {
            Vector3D correspVec = new Vector3D();
            correspVec.sub(pcl1[i], pcl2[corresp[i]]);
            double dist = correspVec.length();

            // System.out.println(dist);

            total += dist;
            if (dist < threshold) {
                DMatrixRMaj p = new DMatrixRMaj(1, 3);
                DMatrixRMaj q = new DMatrixRMaj(1, 3);
                p.set(0, 0, pcl1[i].getX());
                p.set(0, 1, pcl1[i].getY());
                p.set(0, 2, pcl1[i].getZ());
                q.set(0, 0, pcl2[corresp[i]].getX());
                q.set(0, 1, pcl2[corresp[i]].getY());
                q.set(0, 2, pcl2[corresp[i]].getZ());
                CommonOps_DDRM.transpose(p);
                DMatrixRMaj pq = new DMatrixRMaj(3, 3);
                CommonOps_DDRM.mult(p, q, pq);
                CommonOps_DDRM.add(matrixToPack, pq, matrixToPack);
            }
        }
        return total;
    }


    public void transformPointCloud(Point3D[] pcl, RotationMatrix rotation, Vector3D translation) {
        if (rotation != null) {
            for (int i = 0; i < pcl.length; i++) {
                rotation.transform(pcl[i]);
            }
        }
        if (translation != null) {
            for (int i = 0; i < pcl.length; i++) {
                pcl[i].add(translation);
            }
        }
    }

    public void centerPointCloud(Point3D[] pcl1) {
        Point3D centroid = new Point3D(0, 0, 0);

        for (int i = 0; i < pcl1.length; i++) {
            centroid.add(pcl1[i]);
        }
        centroid.scale(1 / (double) pcl1.length);
        for (int i = 0; i < pcl1.length; i++) {
            pcl1[i].sub(centroid);
        }
    }


    public void getRotation(DMatrixRMaj crossCovariance, RotationMatrix rotationToPack) {
        DMatrixRMaj U = new DMatrixRMaj(3, 3);
        DMatrixRMaj Vt = new DMatrixRMaj(3, 3);
        DGrowArray D = new DGrowArray(3);
        SingularOps_DDRM.svd(crossCovariance, U, D, Vt);
        DMatrixRMaj R = new DMatrixRMaj(3, 3);
        CommonOps_DDRM.mult(U, Vt, R);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotationToPack.set(R);
            }
        }
    }
}
