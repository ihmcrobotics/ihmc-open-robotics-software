package us.ihmc.sensorProcessing.concaveHull;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Scanner;

import org.opensphere.geometry.algorithm.ConcaveHull;
import org.opensphere.geometry.triangulation.model.Edge;
import org.opensphere.geometry.triangulation.model.Triangle;
import org.opensphere.geometry.triangulation.model.Vertex;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;

public class Delaunay {
    	
	public static void main(String args[]) throws FileNotFoundException{
		Scanner sc = new Scanner(new File("plane.txt"));
		
		BoundConcavePolygon conversion2d = new BoundConcavePolygon();
		Coordinate[] points = new Coordinate[3000];
		List<Point3D_F64> points3d= new ArrayList<Point3D_F64>();
		int p=0;
		 while(sc.hasNextLine()){
			 String line=sc.nextLine();
			 String[] tokens= line.split("\\s");
			// System.out.println(tokens[0]+" "+tokens[1]+" "+tokens[2]);
			 points3d.add(new Point3D_F64(Double.parseDouble(tokens[0]),Double.parseDouble(tokens[1]),Double.parseDouble(tokens[2])));
			 
			 //points[p]= new Coordinate(Double.parseDouble(tokens[0]),Double.parseDouble(tokens[1]),Double.parseDouble(tokens[2]));
			 p++;
		 }
		
		 PlaneGeneral3D_F64 plane = new PlaneGeneral3D_F64(0,1,0,2);
		 conversion2d.process(plane, points3d);
		 List<Point2D_F64> points2d =conversion2d.getPoints2D();
		 System.out.println(points2d.size()+"total 2d points");
		 for(int i=0;i<points2d.size();i++){
			 Point2D_F64 po=points2d.get(i);
			 points[i]= new Coordinate(po.x,po.y);
			 System.out.println(points[i]);
		 }
		 

		GeometryCollection geometry = new GeometryFactory().createMultiPoint(points);
		ConcaveHull c = new ConcaveHull(geometry, 0.7);
		Geometry concaveHull = c.getConcaveHull();
		System.out.println("the number of triangles "+c.triangles.size());
		// testing the triangles
		HashMap<Integer,Vertex> verts = new HashMap<Integer, Vertex>();
		Vertex ov,ev;
		for(Triangle triangle : c.triangles.values()){
		    List<Edge> edges = triangle.getEdges();
		    for(Edge e: edges){
		       ov = e.getOV();
		       verts.put(Integer.valueOf(ov.getId()),ov);
		       ev = e.getEV();
		       verts.put(Integer.valueOf(ev.getId()), ev);
		    }
		}
		System.out.println("the number of vertices "+verts.keySet().toArray());
		
//		System.out.println(concaveHull.toText());
//		System.out.println(concaveHull.getGeometryType());
		    Coordinate[] boundary = concaveHull.getCoordinates();
	    
	    //new code
	    PrintStream out1 = new PrintStream("poly.txt");
		for(int i=0;i<boundary.length;i++){
			String str="";
				Point3D_F64 p3= conversion2d.convertTo3D(boundary[i].x, boundary[i].y, conversion2d.getCenter());
			str=p3.x+" "+p3.y+" "+p3.z;
			out1.println(str);
			
				
		}
		out1.close();
		
	}

	

	
	
}
