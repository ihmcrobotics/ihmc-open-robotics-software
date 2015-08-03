package us.ihmc.sensorProcessing.pointClouds.shape;


import georegression.metric.Distance3D_F64;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector3D_F64;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.vecmath.Point3d;

import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.utilities.lidar.LidarScan;
import us.ihmc.utilities.lidar.LidarScanParameters;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.PointCloudShapeFinder.Shape;

import com.jme3.app.SimpleApplication;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;

public class LidarOffsetCalculator extends SimpleApplication
{
   Random rand = new Random();

   public String fileName;

   public static void main(String[] args)
   {
      //LidarOffsetCalculator test1 = new LidarOffsetCalculator("lidar_dump_1384213072427.txt");
      LidarOffsetCalculator test1 = new LidarOffsetCalculator("lidar_dump_1384211687790.txt");
      test1.start();
   }

   public LidarOffsetCalculator(String fileName)
   {
      this.fileName = fileName;
   }

   @Override
   public void simpleInitApp()
   {
      //REAL
      float crc = -.0010908f;
      //crc = 0;
      LidarScanParameters param = new LidarScanParameters(1081, -2.356194f+crc, 2.356194f+crc, 0, 0, 0, 0);
      
      //SCS
      //LidarScanParameters param = new LidarScanParameters(720, -1.570796f, 1.570796f, 0, 0, 1, 0, 0, 0, 0, 0, false);

      List<Point3D_F64>[] clouds = loadPointCloud((int)(40*30), param, 1, true);

      render(clouds);
   }

   private void render(List<Point3D_F64>[] clouds)
   {
      int total = 0;
      for (List<Point3D_F64> cloud : clouds)
      {
         total += cloud.size();
      }

//      List<Point3D_F64> greenPoints = new ArrayList<Point3D_F64>();
     List<Point3D_F64> points2 = new ArrayList<Point3D_F64>();
      List<List<Point3D_F64>> points = new ArrayList<List<Point3D_F64>>();
      List<Vector3f> vectors = new ArrayList<Vector3f>();
      List<ColorRGBA> colors = new ArrayList<ColorRGBA>();

      int index = 0;
      for (int i = 0; i < clouds.length; i++)
      {
         int c = Color.HSBtoRGB((i / (float) clouds.length), 1.0f, i == 2 ? 1.0f : 1.0f);
         ColorRGBA color = new ColorRGBA(((c >> 16) & 0xFF) / 256.0f, ((c >> 8) & 0xFF) / 256.0f, ((c >> 0) & 0xFF) / 256.0f, 1.0f);
         System.out.println(clouds[i].size());
         points.add(new ArrayList<Point3D_F64>());
         for (Point3D_F64 p : clouds[i])
         {
            
        	 Vector3f vec=new Vector3f((float) p.x, (float) p.y, (float) p.z);
        	 if(vec.length() <2){ //&& vec.getZ()> -0.3f && vec.getZ() < 1.0f){
                 points.get(i).add(p);
                 vectors.add(vec);
                 points2.add(p);
                 colors.add(color);
        	 }
            
         }
      }
      Node zUpNode = new Node();
      zUpNode.setLocalRotation(JMEGeometryUtils.getRotationFromJMEToZupCoordinates());
      
      ShapesFromPointCloudFileApp sp = new ShapesFromPointCloudFileApp("none");
      ShapeTranslator translator = new ShapeTranslator(this);
      
      //calculateDistanceOffset(points2,zUpNode,sp,translator);
     
      calculateAngularOffset(points,zUpNode,sp,translator);
      
      
     
     
      PointCloud generator = new PointCloud(assetManager);

      try
      {
         rootNode.attachChild(zUpNode);
         zUpNode.attachChild(generator.generatePointCloudGraph(vectors, colors));
      }
      catch (Exception e)
      {
         this.handleError(e.getMessage(), e);
      }

      cam.setFrustumPerspective(45.0f, ((float) cam.getWidth()) / ((float) cam.getHeight()), 0.05f, 100.0f);
      cam.setLocation(new Vector3f(0, 0, -5));
      cam.lookAtDirection(Vector3f.UNIT_Z, Vector3f.UNIT_Y);
      cam.update();
      flyCam.setMoveSpeed(25);
   }

   
private void calculateAngularOffset(List<List<Point3D_F64>> points,Node zUpNode, ShapesFromPointCloudFileApp sp, ShapeTranslator translator) {
	// TODO Auto-generated method stub
	List<PointCloudShapeFinder.Shape> shapesRed = sp.run_ransac(points.get(0),false);
    List<PointCloudShapeFinder.Shape> shapesGreen = sp.run_ransac(points.get(1),false);
  
    renderPlane(zUpNode, translator, shapesGreen,ColorRGBA.Green);     
    renderPlane(zUpNode, translator, shapesRed,ColorRGBA.Red);
    
   double threshold=1.0; 
   List<PointCloudShapeFinder.Shape[]> finalShapes = new ArrayList<PointCloudShapeFinder.Shape[]>();
   double planeAngle,min,minDist,avgAngle=0,avgDist=0;
   PointCloudShapeFinder.Shape temps2;
   for(PointCloudShapeFinder.Shape s1 : shapesGreen){
  	 	
  	 min=2* Math.PI;
  	 temps2 = s1;
  	 minDist=threshold;
  	for(PointCloudShapeFinder.Shape s2 : shapesRed){
  	
  		 planeAngle = angle(s1,s2) ;
  		
  		 if(planeAngle< min){
  			 	
  			 Point3D_F64 g = calculateMean(s1.points);
  	    	 double dist = calculateMean(s2.points).distance(g); 
  	    	 
  	    	 if(dist<=threshold){
  				 min = planeAngle;
      			 temps2 = s2;
      			 minDist = dist;
  	    	 }
  			
  		 }
  	 }
  
  	 System.out.println("min Angle"+Math.toDegrees(min));
		 System.out.println("distance"+ minDist);
  	 if(temps2!=s1 && Math.toDegrees(min)<10){
  		 avgAngle+=Math.toDegrees(min);
  		 avgDist+=minDist;
  		 System.out.println("min Angle"+Math.toDegrees(min));
  		 System.out.println("distance"+ minDist);
  	 PointCloudShapeFinder.Shape[] arr = new PointCloudShapeFinder.Shape[2]; 
		 arr[0] = s1;
		 arr[1] = temps2;
		 finalShapes.add(arr);
  	 }
   }
   avgAngle=avgAngle/finalShapes.size();
   System.out.println("Avergae angle offset"+avgAngle);
   avgDist=avgDist/finalShapes.size();
   System.out.println("Avergae angle offset"+avgDist);
   
   renderPlane(zUpNode, translator, finalShapes,null);   
   
}

private void calculateDistanceOffset(List<Point3D_F64> points2, Node zUpNode,ShapesFromPointCloudFileApp sp, ShapeTranslator translator) {
	// TODO Auto-generated method stub
	 List<PointCloudShapeFinder.Shape> shapes = sp.run_ransac(points2,true);
     renderPlane(zUpNode, translator, shapes, ColorRGBA.Green);
     System.out.println(shapes.size());
     double distance = 0.0;
     for(Point3D_F64 p : shapes.get(0).points){
   	  distance+=Math.abs(Distance3D_F64.distance((PlaneGeneral3D_F64)shapes.get(0).parameters, p));
   	  
     }
     distance/= shapes.get(0).points.size();
     System.out.println("distance offset"+ 2*distance);
}

private <E> void renderPlane(Node zUpNode, ShapeTranslator translator, List<E> shapes,ColorRGBA color) {
	float hue=0;
	for(E s : shapes){
     	 int c1 = Color.HSBtoRGB(hue, 1.0f, 1.0f);
          hue += (1.0 / shapes.size());
          final ColorRGBA color1 = new ColorRGBA(((c1 >> 16) & 0xFF) / 256.0f, ((c1 >> 8) & 0xFF) / 256.0f, ((c1 >> 0) & 0xFF) / 256.0f, 1.0f);
		
          if(s.getClass().isArray()){
        	  PointCloudShapeFinder.Shape[] plane = (PointCloudShapeFinder.Shape[])s;
        	  translator.createPolygon(plane[0], color1, zUpNode);
        	  translator.createPolygon(plane[1], color1, zUpNode);
        	  Point3D_F64 m1= calculateMean(plane[0].points);
        	  Point3D_F64 m2 = calculateMean(plane[1].points);
        	  zUpNode.attachChild(translator.drawLine(new Vector3f((float)m1.x,(float)m1.y,(float)m1.z), new Vector3f((float)m2.x,(float)m2.y,(float)m2.z)));
        	 
          }
          else{
        		 translator.createPolygon( (PointCloudShapeFinder.Shape)s, color, zUpNode);
          }
     
     	
     	
      }
	
}
   
private Point3D_F64 calculateMean(List<Point3D_F64> points) {
	// TODO Auto-generated method stub
	double meanX=0;
	double meanY=0;
	double meanZ=0;
	for( int i = 0; i < points.size(); i++ ) {
		Point3D_F64 p = points.get(i);

		meanX += p.x;
		meanY += p.y;
		meanZ += p.z;
	}

	// center of the points in the 2D coordinate system
	meanX /= points.size();
	meanY /= points.size();
	meanZ /= points.size();
			
	return new Point3D_F64(meanX, meanY, meanZ);
}
   private double angle(Shape s1, Shape s2) {
	// TODO Auto-generated method stub
	   PlaneGeneral3D_F64 p1 =(PlaneGeneral3D_F64) s1.parameters;
	   PlaneGeneral3D_F64 p2 = (PlaneGeneral3D_F64) s2.parameters;
	   Vector3D_F64 norm1 = new Vector3D_F64(p1.A, p1.B,p1.C);
	   Vector3D_F64 norm2 = new Vector3D_F64(p2.A, p2.B, p2.C);
	   norm1.normalize();
	   norm2.normalize();
	   double angInRadians = Math.acos(norm1.dot(norm2));
	   double angInDegrees = Math.toDegrees(angInRadians);
	   if (angInDegrees<90 || angInDegrees>270)
		   return angInRadians;
	   else
		   return (Math.PI - angInRadians);
	   
	
}
   
   private void getPlanes(){
	   
   }

private List<Point3D_F64>[] loadPointCloud(int maxScans, LidarScanParameters params, int mod, boolean half)
   {
      List<Point3D_F64>[] clouds = new ArrayList[3];
      for (int i = 0; i < clouds.length; i++)
         clouds[i] = new ArrayList<Point3D_F64>();

      try
      {
         String file = new Scanner(new BufferedReader(new FileReader(fileName))).useDelimiter("\\Z").next();

         RigidBodyTransform start, end;
         float distance;

         List<String> allMatches = new ArrayList<String>();
         Matcher m = Pattern.compile("-?[0-9]+.[0-9]+(E-[0-9])?").matcher(file);
         while (m.find())
         {
            allMatches.add(m.group());
         }
         double[] doubles = new double[allMatches.size()];
         for (int i = 0; i < doubles.length; i++)
            doubles[i] = Float.valueOf(allMatches.get(i));

         int i = 0;
         int scans = 0;
         while (scans < maxScans && i < doubles.length - 1000)
         {
            scans++;

            if (scans % mod != 0)
               continue;

            double[] mx = new double[16];
            for (int j = 0; j < 16; j++)
               mx[j] = doubles[i++];
            start = new RigidBodyTransform(mx);

            mx = new double[16];
            for (int j = 0; j < 16; j++)
               mx[j] = doubles[i++];
            end = new RigidBodyTransform(mx);

            float[] ranges = new float[params.pointsPerSweep];
            for (int j = 0; j < params.pointsPerSweep; j++)
            {
               distance = (float) doubles[i++];
               ranges[j] = distance;
            }

            LidarScan scan = new LidarScan(params, start, end, ranges);

            for (int j = 0; j < scan.size(); j++)
            {
               if (.5 < scan.getRange(j) && scan.getRange(j) < 30.0)
               {
                  Point3d p = scan.getPoint(j);
                  if (!half || j > scan.size() / 2)
                     clouds[0].add(new Point3D_F64(p.x, p.y, p.z));
                  else
                     clouds[1].add(new Point3D_F64(p.x, p.y, p.z));
               }
            }
         }
      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      return clouds;
   }
}
