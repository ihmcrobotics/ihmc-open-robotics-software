package us.ihmc.ihmcPerception.lineSegmentDetector;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.VideoPacket;
import javafx.scene.shape.Sphere;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.ximgproc.FastLineDetector;
import org.opencv.ximgproc.Ximgproc;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.ui.JavaFXPlanarRegionsViewer;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.Ros2Node;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static java.lang.StrictMath.acos;
import static java.lang.StrictMath.pow;

public class LineSegmentEstimator {


    private boolean prevImgFilled = false;
    private boolean currentVideoPacketFilled = false;
    private boolean currentPlanarRegionsListMessageFilled = false;
    private int code = 0;

    private VideoPacket currentVideoPacket;
    private PlanarRegionsListMessage currentPlanarRegionsListMessage;

    private Sphere sensorNode;

    private Mat curImg;
    private Mat prevImg;
    private Mat curLines;
    private Mat prevLines;
    private ArrayList<LineMatch> correspLines;

    private CameraPinholeBrown camIntrinsics;

    private JavaFXPlanarRegionsViewer planarRegionsViewer;
    private Ros2Node ros2Node;
    private JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
    private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

    public void setPlanarRegionsViewer(JavaFXPlanarRegionsViewer planarRegionsViewer) {
        this.planarRegionsViewer = planarRegionsViewer;
    }

    public void setSensorNode(Sphere sensorNode) {
        this.sensorNode = sensorNode;
    }

    public void execFootstepPlan(){
        FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
        boolean loadResult = logLoader.load(new File("/home/quantum/.ihmc/logs/20200821_172044151_FootstepPlannerLog/"));
        if (!loadResult)
        {
            LogTools.error("Couldn't find file");
            return;
        }
        FootstepPlannerLog log = logLoader.getLog();
        FootstepPlanningToolboxOutputStatus statusPacket = log.getStatusPacket();
        FootstepDataListMessage footstepDataList = statusPacket.getFootstepDataList();

        String robotName = "Atlas";

        IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher = ROS2Tools.createPublisherTypeNamed(this.ros2Node,
                FootstepDataListMessage.class,
                ROS2Tools.getControllerInputTopic(robotName));
        footstepDataListPublisher.publish(footstepDataList);
    }

    public LineSegmentEstimator() {
        // System.out.println(System.getProperty("java.library.path"));
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        // NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
        this.ros2Node = ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
        this.registerCallbacks(ros2Node);
        executorService.scheduleAtFixedRate(this::mainUpdate, 0, 32, TimeUnit.MILLISECONDS);

        // execFootstepPlan();


    }

    public void videoPacketCallback(VideoPacket message) {
        currentVideoPacketFilled = true;
        currentVideoPacket = message;
    }

    public void planarRegionsListCallback(PlanarRegionsListMessage message) {
        currentPlanarRegionsListMessageFilled = true;
        currentPlanarRegionsListMessage = message;
        // planarRegionsViewer.submitPlanarRegions(message);
    }

    public void registerCallbacks(Ros2Node ros2Node) {
        new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REA.withOutput(), this::planarRegionsListCallback);
        new ROS2Callback<>(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT, this::videoPacketCallback);
    }

    public void mainUpdate() {

        if (currentVideoPacketFilled) {
            BufferedImage bufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(this.currentVideoPacket.getData().toArray());
            Mat mat = new Mat(bufferedImage.getHeight(), bufferedImage.getWidth(), CvType.CV_8UC3);
            byte[] data = ((DataBufferByte) bufferedImage.getRaster().getDataBuffer()).getData();
            mat.put(0, 0, data);
            this.curImg = mat;
            if (!(this.prevImgFilled)) {
                prevImgFilled = true;
                this.prevImg = this.curImg;
                this.prevLines = getFLDLinesFromImage(this.prevImg);
                camIntrinsics = HumanoidMessageTools.toIntrinsicParameters(this.currentVideoPacket.intrinsic_parameters_);


                currentVideoPacketFilled = false;
            } else {
                // System.out.println(camIntrinsics.height + "\t" + camIntrinsics.width);
                // System.out.println("Processing " + prevImgFilled);
                this.processImages();

                this.prevImgFilled = true;
                this.prevImg = this.curImg;
                this.prevLines = this.curLines;
            }


            // sensorNode.setTranslateX(currentVideoPacket.position_.getX());
            // sensorNode.setTranslateY(currentVideoPacket.position_.getY());
            // sensorNode.setTranslateZ(currentVideoPacket.position_.getZ());

        }

        if(currentPlanarRegionsListMessageFilled){
            // LogTools.info(currentPlanarRegionsListMessage.toString());
            currentPlanarRegionsListMessageFilled = false;

        }
        if(this.code==81) System.exit(0);

    }

    public void processImages() {

        float startTime = System.nanoTime();

        Mat dispImage = new Mat();
        this.curLines = getFLDLinesFromImage(this.curImg);
        this.correspLines = matchFLDLines(this.prevLines, this.curLines);

        for (int i = 0; i < curLines.rows(); i++) {
            double[] val = curLines.get(i, 0);
            Imgproc.line(this.curImg, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(0, 255, 255), 2);
        }

        // List<Mat> src = Arrays.asList(prevImg, curImg);
        // Core.hconcat(src, dispImage);
        // for (LineMatch lm : correspLines) {
        //     Imgproc.line(dispImage, new Point(lm.prevLine[0], lm.prevLine[1]),
        //             new Point(lm.curLine[0] + prevImg.cols(), lm.curLine[1]),
        //             new Scalar(255, 50, 50), 2);
        //     Imgproc.line(dispImage, new Point(lm.prevLine[2], lm.prevLine[3]),
        //             new Point(lm.curLine[2] + prevImg.cols(), lm.curLine[3]),
        //             new Scalar(255, 50, 50), 2);
        // }

        PlanarRegionsList newRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(this.currentPlanarRegionsListMessage);

        Mat curImgSegment = new Mat();
        curImg.copyTo(curImgSegment);

        for (PlanarRegion region : newRegions.getPlanarRegionsAsList()){

            RigidBodyTransform tfLocalToWorld = new RigidBodyTransform();
            region.getTransformToWorld(tfLocalToWorld);

            QuaternionBasedTransform tfWorldToCamera = new QuaternionBasedTransform(currentVideoPacket.orientation_,currentVideoPacket.position_);

            List<Point2D> concaveHull = region.getConcaveHull();

            ArrayList<Point> pointList = new ArrayList<>();

            Point2D regionMidPoint = new Point2D(0,0);
            int regionSize = 0;

            for(int i = 0; i<concaveHull.size(); i++){
                Point3D p3d = new Point3D(concaveHull.get(i).getX(), concaveHull.get(i).getY(), 0);
                p3d.applyTransform(tfLocalToWorld);
                p3d.applyInverseTransform(tfWorldToCamera);
                Point3D tfdP3d = new Point3D(-p3d.getY(), -p3d.getZ(), p3d.getX());
                if(tfdP3d.getZ() >= 0){
                    double px = camIntrinsics.cx + camIntrinsics.fx * tfdP3d.getX() / tfdP3d.getZ();
                    double py = camIntrinsics.cy + camIntrinsics.fy * tfdP3d.getY() / tfdP3d.getZ();
                    regionMidPoint.add(px, py);
                    regionSize += 1;
                    pointList.add(new Point(px,py));
                }
            }

            if (pointList.size() > 2){



                MatOfPoint points = new MatOfPoint();
                points.fromList(pointList);
                List<MatOfPoint> ppt = new ArrayList<>();
                ppt.add(points);
                regionMidPoint.scale(1/(float)regionSize);

                if(regionMidPoint.getX() < curImg.width() && regionMidPoint.getY() < curImg.height() && regionMidPoint.getX() >= 0 && regionMidPoint.getY() >= 0 ){
                    // Mat mask = Mat.zeros(curImg.rows() + 2, curImg.cols() + 2, CvType.CV_8U);
                    // Imgproc.floodFill(curImgSegment, mask, new Point((int)regionMidPoint.getX(), (int)regionMidPoint.getY()),
                    //         new Scalar(100 + region.getRegionId()*123 % 155, 100 + region.getRegionId()*321 % 155, 100 + region.getRegionId()*135 % 155),
                    //         new Rect(), new Scalar(4,4,4), new Scalar(4,4,4), 4);


                    ArrayList<Point> regionSegment = getSegmentFromLines(curLines, regionMidPoint);

                    for(Point segment : regionSegment){
                        Imgproc.line(curImgSegment, new Point(regionMidPoint.getX(), regionMidPoint.getY()), new Point(segment.x, segment.y),
                                new Scalar(255, 100, 10), 2);
                    }
                    System.out.println("Segment Neighbors: " + regionSegment.size());
                }

                Imgproc.circle(curImgSegment,new Point(regionMidPoint.getX(), regionMidPoint.getY()), 3, new Scalar(255,140,255),-1);
                Imgproc.fillPoly(curImg,ppt,new Scalar(region.getRegionId()*123 % 255, region.getRegionId()*321 % 255, region.getRegionId()*135 % 255) );
                Imgproc.circle(curImg,new Point(regionMidPoint.getX(), regionMidPoint.getY()), 3, new Scalar(255,140,255),-1);
            }
        }

        List<Mat> src = Arrays.asList(curImgSegment, curImg);
        Core.hconcat(src, dispImage);

        float endTime = System.nanoTime();
        // System.out.println("Time:" + (int) ((endTime - startTime) * (1e-6)) + " ms\tCorresponding Lines:" + correspLines.size());
        Imgproc.resize(dispImage, dispImage, new Size(2400, 1000));
        HighGui.namedWindow("LineEstimator", HighGui.WINDOW_AUTOSIZE);
        HighGui.imshow("LineEstimator", dispImage);
        this.code = HighGui.waitKey(32);
        if(this.code == 81) System.exit(0);
    }

    private static ArrayList<Point> getSegmentFromLines(Mat curLines, Point2D centroid){
        ArrayList<Point> segment = new ArrayList<>();
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
            if(dist < 3000){
                // segment.add(new Point(line[0],line[1]));
                // segment.add(new Point(line[2],line[3]));
                segment.add(new Point(m.getX(), m.getY()));
            }
        }
        return segment;
    }

    private static ArrayList<LineMatch> matchFLDLines(Mat prevFLDLines, Mat FLDlines) {
        ArrayList<LineMatch> corresp = new ArrayList<>();
        double angleCost, midPointCost, lengthCost = 0;
        double angleThreshold = 50, midPointThreshold = 100, lengthThreshold = 1200;
        for (int i = 0; i < prevFLDLines.rows(); i++) {
            double[] pl = prevFLDLines.get(i, 0);
            for (int j = 0; j < FLDlines.rows(); j++) {
                double[] cl = FLDlines.get(j, 0);
                angleCost = calcAngleCost(pl, cl);
                midPointCost = calcMidPointCost(pl, cl);
                lengthCost = calcLengthCost(pl, cl);
                if (angleCost < angleThreshold
                        && midPointCost < midPointThreshold
                    // &&  lengthCost < lengthThreshold
                ) {
                    // printf("%.2lf\t%.2lf\t%.2lf\n",angleCost, midPointCost, lengthCost);
                    LineMatch lm = new LineMatch(pl, cl, angleCost, midPointCost, lengthCost);
                    corresp.add(lm);
                }
            }
        }
        return corresp;
    }

    private static double calcAngleCost(double[] l1, double[] l2) {
        Vector2D a = new Vector2D(l1[0], l1[1]);
        Vector2D b = new Vector2D(l1[2], l1[3]);
        Vector2D c = new Vector2D(l2[0], l2[1]);
        Vector2D d = new Vector2D(l2[2], l2[3]);
        Vector2D l1_hat = new Vector2D();
        l1_hat.sub(a, b);
        Vector2D l2_hat = new Vector2D();
        l2_hat.sub(c, d);
        l1_hat.normalize();
        l2_hat.normalize();
        return pow(acos(l1_hat.dot(l2_hat)) * 360 / EuclidCoreTools.TwoPI, 2);
    }

    private static double calcMidPointCost(double[] l1, double[] l2) {
        Vector2D a = new Vector2D(l1[0], l1[1]);
        Vector2D b = new Vector2D(l1[2], l1[3]);
        Vector2D c = new Vector2D(l2[0], l2[1]);
        Vector2D d = new Vector2D(l2[2], l2[3]);
        Vector2D m1 = new Vector2D();
        m1.add(a, b);
        m1.scale(0.5);
        Vector2D m2 = new Vector2D();
        m2.add(c, d);
        m2.scale(0.5);
        Vector2D p = new Vector2D();
        p.sub(m1, m2);
        return pow(p.length(), 2);
    }

    private static double calcLengthCost(double[] l1, double[] l2) {
        Vector2D a = new Vector2D(l1[0], l1[1]);
        Vector2D b = new Vector2D(l1[2], l1[3]);
        Vector2D c = new Vector2D(l2[0], l2[1]);
        Vector2D d = new Vector2D(l2[2], l2[3]);
        Vector2D ab = new Vector2D();
        Vector2D cd = new Vector2D();
        ab.sub(a, b);
        cd.sub(c, d);
        return pow(ab.length() - cd.length(), 2);
    }

    private static Mat getFLDLinesFromImage(Mat image) {
        Mat lines = new Mat();
        Mat gray = new Mat();
        Imgproc.cvtColor(image, gray, Imgproc.COLOR_BGR2GRAY);

        FastLineDetector fld = Ximgproc.createFastLineDetector(20, 1.41421356f,
                50, 50, 3, true);
        fld.detect(gray, lines);
        return lines;
    }

    public static Mat getLinesFromImage(Mat image) {
        Mat frame = preProcessImage(image);
        Mat edges = getCannyEdges(frame);
        return getHoughLinesFromEdges(edges);
    }

    private static Mat preProcessImage(Mat frame) {
        Mat gray = new Mat();
//        Imgproc.pyrDown(frame,frame);
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.medianBlur(gray, gray, 5);
        Imgproc.GaussianBlur(gray, gray, new Size(3, 3), 20);
        return gray;
    }

    private static Mat getCannyEdges(Mat gray) {
        Mat edges = new Mat();
        Imgproc.Canny(gray, edges, 60, 60 * 3, 3, true);
        Mat cannyColor = new Mat();
        Imgproc.cvtColor(edges, cannyColor, Imgproc.COLOR_GRAY2BGR);
        return edges;
    }

    private static Mat getHoughLinesFromEdges(Mat edges) {
        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 70, 30, 10);
        return lines;
    }

    private static Mat getImage(String path) {
        return Imgcodecs.imread(path);
    }


    public static void main(String[] args) {
        new LineSegmentEstimator();
    }

}
