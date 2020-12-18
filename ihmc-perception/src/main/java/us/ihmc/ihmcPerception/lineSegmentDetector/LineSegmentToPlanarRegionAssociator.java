package us.ihmc.ihmcPerception.lineSegmentDetector;

import boofcv.struct.calib.CameraPinholeBrown;
import com.jme3.math.LineSegment;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import org.lwjgl.opengl.Display;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

public class LineSegmentToPlanarRegionAssociator
{
   private CameraPinholeBrown camIntrinsics;
   private Quaternion cameraOrientation;
   private Point3D cameraPosition;

   private PoseReferenceFrame regionFrame;
   private PoseReferenceFrame cameraFrame;

   private Mat curLines;

   public LineSegmentToPlanarRegionAssociator(ReferenceFrame sensorFrame)
   {
      regionFrame = new PoseReferenceFrame("RegionFrame", ReferenceFrame.getWorldFrame());
      cameraFrame = new PoseReferenceFrame("CameraFrame", sensorFrame);
//      cameraFrame.setOrientationAndUpdate(new RotationMatrix(0, -1, 0, 0, 0, -1, 1, 0, 0));
   }

   public LineSegmentToPlanarRegionAssociator()
   {

   }

   public void setIntrinsics(CameraPinholeBrown intrinsics)
   {
      this.camIntrinsics = intrinsics;
   }

   public void setCameraOrientation(Quaternion cameraOrientation)
   {
      this.cameraOrientation = cameraOrientation;
   }

   public void setCameraPosition(Point3D cameraPosition)
   {
      this.cameraPosition = cameraPosition;
   }

   public void associateInImageSpace(PlanarRegionsListMessage currentPlanarRegionsListMessage){
      PlanarRegionsList newRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(currentPlanarRegionsListMessage);
//      LogTools.info("Planar Regions: {}", newRegions.getNumberOfPlanarRegions());
      for (PlanarRegion region : newRegions.getPlanarRegionsAsList())
      {
         ArrayList<Point> pointList = new ArrayList<>();
         Point2D regionMidPoint = new Point2D(0, 0);

         if (region.getConvexHull().getArea() * 1000 > 50)
         {
            PlanarSegment regionBasedSegment = new PlanarSegment(region.getRegionId());
            projectPlanarRegionUsingCameraPose(region, regionBasedSegment);
            DisplayTools.drawPolygon(regionBasedSegment, 1);

            PlanarSegment lineBasedSegment = new PlanarSegment(region.getRegionId());
            associateLinesToSegment(lineBasedSegment, regionBasedSegment);
            DisplayTools.drawPolygon(lineBasedSegment, 0);
         }
      }

   }

   public ArrayList<Point> projectPlanarRegion(PlanarRegion region, Point2D regionMidPoint)
   {
      regionFrame.setPoseAndUpdate(region.getTransformToWorld());

      List<Point2D> concaveHull = region.getConcaveHull();

      ArrayList<Point> pointList = new ArrayList<>();

      int regionSize = 0;

      LogTools.info(camIntrinsics);
      LogTools.info(cameraFrame.getTransformToParent());
      for (int i = 0; i < concaveHull.size(); i++)
      {
         FramePoint3D vertex = new FramePoint3D(regionFrame, concaveHull.get(i).getX(), concaveHull.get(i).getY(), 0);
         vertex.changeFrame(cameraFrame);


         // (-Y, -Z, X)
         Point3D tfdP3d = new Point3D(-vertex.getY(), -vertex.getZ(), vertex.getX());
         LogTools.info("Point3D CamFrame: {} {} {}", tfdP3d.getX32(), tfdP3d.getY32(), tfdP3d.getZ32());

         if (tfdP3d.getZ() >= 0)
         {
            double px = camIntrinsics.cx + camIntrinsics.fx * tfdP3d.getX() / tfdP3d.getZ();
            double py = camIntrinsics.cy + camIntrinsics.fy * tfdP3d.getY() / tfdP3d.getZ();
            regionMidPoint.add(px, py);
            regionSize += 1;
            pointList.add(new Point(px, py));
            LogTools.info("Image Points: {} {} {}", regionSize, px, py);
         }
      }
      regionMidPoint.scale(1 / (double) regionSize);
      return pointList;
   }

   public void projectPlanarRegionUsingCameraPose(PlanarRegion region, PlanarSegment segmentToPack)
   {
      RigidBodyTransform tfLocalToWorld = new RigidBodyTransform();
      region.getTransformToWorld(tfLocalToWorld);
      QuaternionBasedTransform tfWorldToCamera = new QuaternionBasedTransform(cameraOrientation, cameraPosition);
      List<Point2D> concaveHull = region.getConcaveHull();
      Point2D centroid = new Point2D();

      for (int i = 0; i < concaveHull.size(); i++) {
         Point3D p3d = new Point3D(concaveHull.get(i).getX(), concaveHull.get(i).getY(), 0);
         p3d.applyTransform(tfLocalToWorld);
         p3d.applyInverseTransform(tfWorldToCamera);
         Point3D tfdP3d = new Point3D(-p3d.getY(), -p3d.getZ(), p3d.getX());
         if (tfdP3d.getZ() >= 0) {
//            LogTools.info("Region Point:{} {} {}", camIntrinsics.cx, camIntrinsics.cy, tfdP3d);
            double px = camIntrinsics.cx + camIntrinsics.fx * tfdP3d.getX() / tfdP3d.getZ();
            double py = camIntrinsics.cy + camIntrinsics.fy * tfdP3d.getY() / tfdP3d.getZ();
            centroid.add(px, py);
            segmentToPack.getVertices().add(new Point2D(px, py));
         }
      }
      centroid.scale(1 / (double) segmentToPack.getVertices().size());
      segmentToPack.updateArea();
      segmentToPack.setCentroid(centroid);
   }



   public void associateLinesToSegment(PlanarSegment lineBasedSegment, PlanarSegment regionBasedSegment)
   {

      if (regionBasedSegment.getVertices().size() > 2)
      {
         if (regionBasedSegment.getCentroid().getX() < camIntrinsics.getWidth() && regionBasedSegment.getCentroid().getY() < camIntrinsics.getHeight()
             && regionBasedSegment.getCentroid().getX() >= 0 && regionBasedSegment.getCentroid().getY() >= 0)
         {
            getSegmentFromLines(curLines, regionBasedSegment, lineBasedSegment);
            lineBasedSegment.setCentroid(regionBasedSegment.getCentroid());
            lineBasedSegment.updateOrder();
            lineBasedSegment.updateArea();
            filterSegmentWithRegion(lineBasedSegment, regionBasedSegment);
            lineBasedSegment.updateOrder();
            lineBasedSegment.updateArea();
         }

      }
   }

   private void filterSegmentWithRegion(PlanarSegment lineBasedSegment, PlanarSegment regionBasedSegment)
   {
      ArrayList<Point2D> regionPoints = regionBasedSegment.getVertices();
      ArrayList<Point2D> segmentPoints = lineBasedSegment.getVertices();
      LogTools.info("Segment Points]: {} {}", regionPoints.size(), segmentPoints.size());
      int lbspCount = 0;
      for(int i = 0; i<regionPoints.size()-1; i++){
         lbspCount %= segmentPoints.size();
         int rA = (int)(Math.toDegrees(Math.atan2(regionPoints.get(i).getX() - regionBasedSegment.getCentroid().getX(), regionPoints.get(i).getY() - regionBasedSegment.getCentroid().getY())) + 360) % 360;
         int rB = (int)(Math.toDegrees(Math.atan2(regionPoints.get(i+1).getX() - regionBasedSegment.getCentroid().getX(), regionPoints.get(i+1).getY() - regionBasedSegment.getCentroid().getY())) + 360) % 360;
         int lX = (int)(Math.toDegrees(Math.atan2(segmentPoints.get(lbspCount).getX() - regionBasedSegment.getCentroid().getX(), segmentPoints.get(lbspCount).getX() - regionBasedSegment.getCentroid().getX())) + 360) % 360;
         if(lX < rA) lbspCount = (lbspCount + 1) % segmentPoints.size();
         else if(rA < lX && lX < rB){
            Line2D regionLine = new Line2D(regionPoints.get(i), regionPoints.get(i+1));
            Line2D segmentLine = new Line2D(regionBasedSegment.getCentroid(), segmentPoints.get(lbspCount));
            Point2D sectPoint = (Point2D) regionLine.intersectionWith(segmentLine);
            double distSq = sectPoint.distanceSquared(segmentPoints.get(lbspCount));
            if(distSq > 100){
               segmentPoints.remove(lbspCount);
//               LogTools.info("DistSq: {}", distSq);
//               DisplayTools.lineSegment(regionBasedSegment.getCentroid(), segmentPoints.get(lbspCount), new Scalar(0,55,255), 2, 0);
            }
         }
         else if(rB < lX) continue;
         DisplayTools.lineSegment(regionPoints.get(i), regionPoints.get(i+1), new Scalar(255,100,100), 2, 0);
      }
   }

   public void floodFill(Mat img, Point2D seed, int id){
                   Mat mask = Mat.zeros(img.rows() + 2, img.cols() + 2, CvType.CV_8U);
                   Imgproc.floodFill(img, mask, new Point((int)seed.getX(), (int)seed.getY()),
                                     new Scalar(id * 123 % 255, id * 321 % 255, id * 135 % 255),
                           new Rect(), new Scalar(4,4,4), new Scalar(4,4,4), 4);
   }

   public int compare(LineSegment2D a, LineSegment2D b, Point2D centroid)
   {
      double delta = a.distance(centroid) - b.distance(centroid);
      if (delta > 0.00001)
         return 1;
      if (delta < -0.00001)
         return -1;
      return 0;
   }

   public void getSegmentFromLines(Mat curLines, PlanarSegment regionBasedSegment, PlanarSegment segmentToPack)
   {
      PriorityQueue<LineSegment2D> queue = new PriorityQueue<>(10, (a, b) -> compare(a, b, regionBasedSegment.getCentroid()));
      for (int i = 0; i < curLines.rows(); i++)
      {
         double[] line = curLines.get(i, 0);
         LineSegment2D ls = new LineSegment2D(line[0], line[1], line[2], line[3]);
//         LogTools.info("Distance:{}", lineLs.distance(regionBasedSegment.getCentroid()));
         if (ls.distanceSquared(regionBasedSegment.getCentroid()) < 8000 && ls.length() > 30)
         {
            queue.add(ls);
         }
      }

      ArrayList<Line2D> polygon = new ArrayList<Line2D>();
      for (LineSegment2D ls : queue)
      {
            segmentToPack.getVertices().add(new Point2D(ls.getFirstEndpointX(), ls.getFirstEndpointY()));
            segmentToPack.getVertices().add(new Point2D(ls.getSecondEndpointX(), ls.getSecondEndpointY()));
      }
   }

   public void setCamIntrinsics(CameraPinholeBrown camIntrinsics)
   {
      this.camIntrinsics = camIntrinsics;
   }

   public void setCurLines(Mat curLines)
   {
      this.curLines = curLines;
   }
}
