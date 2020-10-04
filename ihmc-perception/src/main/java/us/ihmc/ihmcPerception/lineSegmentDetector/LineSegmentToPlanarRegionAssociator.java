package us.ihmc.ihmcPerception.lineSegmentDetector;

import boofcv.struct.calib.CameraPinholeBrown;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class LineSegmentToPlanarRegionAssociator {

    private Quaternion cameraOrientation;
    private Point3D cameraPosition;
    private CameraPinholeBrown camIntrinsics;

    private Mat curLines;

    public void loadParams(CameraPinholeBrown intrinsics, Quaternion camOrientation, Point3D camPosition){
        this.camIntrinsics = intrinsics;
        this.cameraOrientation = camOrientation;
        this.cameraPosition = camPosition;
    }

    public ArrayList<Point> projectPlanarRegion(PlanarRegion region, Point2D regionMidPoint) {

        RigidBodyTransform tfLocalToWorld = new RigidBodyTransform();
        region.getTransformToWorld(tfLocalToWorld);

        QuaternionBasedTransform tfWorldToCamera = new QuaternionBasedTransform(cameraOrientation, cameraPosition);

        List<Point2D> concaveHull = region.getConcaveHull();

        ArrayList<Point> pointList = new ArrayList<>();

        int regionSize = 0;

        for (int i = 0; i < concaveHull.size(); i++) {
            Point3D p3d = new Point3D(concaveHull.get(i).getX(), concaveHull.get(i).getY(), 0);
            p3d.applyTransform(tfLocalToWorld);
            p3d.applyInverseTransform(tfWorldToCamera);
            Point3D tfdP3d = new Point3D(-p3d.getY(), -p3d.getZ(), p3d.getX());
            if (tfdP3d.getZ() >= 0) {
                double px = camIntrinsics.cx + camIntrinsics.fx * tfdP3d.getX() / tfdP3d.getZ();
                double py = camIntrinsics.cy + camIntrinsics.fy * tfdP3d.getY() / tfdP3d.getZ();
                regionMidPoint.add(px, py);
                regionSize += 1;
                pointList.add(new Point(px, py));
            }
        }
        return pointList;
    }

    public void drawProjectedRegion(Mat img, ArrayList<Point> pointList, Point2D regionMidPoint, int id){
        if (pointList.size() > 2) {

            MatOfPoint points = new MatOfPoint();
            points.fromList(pointList);
            List<MatOfPoint> ppt = new ArrayList<>();
            ppt.add(points);
            regionMidPoint.scale(1 / (float) pointList.size());

            Imgproc.fillPoly(img,ppt,new Scalar(id*123 % 255, id*321 % 255, id*135 % 255) );
            Imgproc.circle(img,new Point(regionMidPoint.getX(), regionMidPoint.getY()), 3, new Scalar(255,140,255),-1);
        }
    }

    public void drawLineRegionAssociation(Mat img, ArrayList<Point> pointList, Point2D regionMidPoint){
        if (pointList.size() > 2) {

            if (regionMidPoint.getX() < camIntrinsics.getWidth() && regionMidPoint.getY() < camIntrinsics.getHeight() && regionMidPoint.getX() >= 0 && regionMidPoint.getY() >= 0) {
                // Mat mask = Mat.zeros(curImg.rows() + 2, curImg.cols() + 2, CvType.CV_8U);
                // Imgproc.floodFill(curImgSegment, mask, new Point((int)regionMidPoint.getX(), (int)regionMidPoint.getY()),
                //         new Scalar(100 + region.getRegionId()*123 % 155, 100 + region.getRegionId()*321 % 155, 100 + region.getRegionId()*135 % 155),
                //         new Rect(), new Scalar(4,4,4), new Scalar(4,4,4), 4);

                ArrayList<Point2D> regionSegment = getSegmentFromLines(curLines, regionMidPoint);

                for (Point2D segment : regionSegment) {
                    Imgproc.line(img, new Point(segment.getX(), segment.getY()), new Point(regionMidPoint.getX(), regionMidPoint.getY()),
                            new Scalar(255, 100, 10), 2);
                    // Imgproc.line(img, new Point(segment.getSecondEndpointX(), segment.getSecondEndpointY()), new Point(regionMidPoint.getX(), regionMidPoint.getY()),
                    //         new Scalar(255, 100, 10), 2);
                    Imgproc.circle(img, new Point(segment.getX(), segment.getY()), 8, new Scalar(255, 0, 255), -1);
                }
            }
            Imgproc.circle(img,new Point(regionMidPoint.getX(), regionMidPoint.getY()), 3, new Scalar(255,140,255),-1);
        }
    }

    public ArrayList<Point2D> getSegmentFromLines(Mat curLines, Point2D centroid) {
        ArrayList<Point2D> segments = new ArrayList<>();
        for (int i = 0; i < curLines.rows(); i++) {
            double[] line = curLines.get(i, 0);
            Point2D a = new Point2D(line[0], line[1]);
            Point2D b = new Point2D(line[2], line[3]);
            Point2D m = new Point2D();
            m.add(a, b);
            m.scale(0.5);
            Point2D p = new Point2D();
            p.sub(m, centroid);
            double dist = p.distanceFromOriginSquared();
            if (dist < 8000) {
                LineSegment2D lineSeg = new LineSegment2D(a.getX(), a.getY(), b.getX(), b.getY());
                Point2D proj = LineTools.getProjection(lineSeg, centroid);
                segments.add(proj);
            }
        }
        return segments;
    }

    public void setCamIntrinsics(CameraPinholeBrown camIntrinsics) {
        this.camIntrinsics = camIntrinsics;
    }

    public void setCameraOrientation(Quaternion cameraOrientation) {
        this.cameraOrientation = cameraOrientation;
    }

    public void setCameraPosition(Point3D cameraPosition){ this.cameraPosition = cameraPosition; }

    public void setCurLines(Mat curLines) { this.curLines = curLines;}
}
