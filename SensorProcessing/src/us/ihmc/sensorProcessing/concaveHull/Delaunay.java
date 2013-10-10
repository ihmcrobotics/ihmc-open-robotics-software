package us.ihmc.sensorProcessing.concaveHull;
import georegression.struct.plane.PlaneGeneral3D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.*;

import org.apache.poi.hssf.usermodel.HSSFSheet;
import org.apache.poi.hssf.usermodel.HSSFWorkbook;
import org.apache.poi.ss.usermodel.Cell;
import org.apache.poi.ss.usermodel.Row;
import org.opensphere.geometry.algorithm.ConcaveHull;

//import bubo.ptcloud.alg.BoundPlaneRectangle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.CoordinateSequenceFactory;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;

public class Delaunay {
    	
	public static void main(String args[]) throws FileNotFoundException{
		Scanner sc = new Scanner(new File("con.txt"));
		
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
		 
//		Random rand = new Random();
//		for(int i =0;i<100;i++){
//			points[i]=new Coordinate(rand.nextFloat(),rand.nextFloat());
//					
//		}
		GeometryCollection geometry = new GeometryFactory().createMultiPoint(points);
		ConcaveHull c = new ConcaveHull(geometry, 0.7);
		Geometry concaveHull = c.getConcaveHull();
		System.out.println(concaveHull.toText());
		System.out.println(concaveHull.getGeometryType());
		HSSFWorkbook workbook = new HSSFWorkbook();
		HSSFSheet sheet = workbook.createSheet("graph");
		for(int i=0;i<100;i++){
			Row row = sheet.createRow(i);
			Cell cell1 = row.createCell(0);
			cell1.setCellValue(points[i].x);
			Cell cell2 = row.createCell(1);
			cell2.setCellValue(points[i].y);
				
		}
		try
        {
            //Write the workbook in file system
            FileOutputStream out = new FileOutputStream(new File("graph_data.xls"));
            workbook.write(out);
            out.close();
            System.out.println("howtodoinjava_demo.xlsx written successfully on disk.");
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
		HSSFWorkbook workbook2 = new HSSFWorkbook();
		HSSFSheet sheet2 = workbook2.createSheet("polygon");
	    Coordinate[] boundary = concaveHull.getCoordinates();
	    
	    //new code
	    PrintStream out1 = new PrintStream("poly.txt");
		for(int i=0;i<boundary.length;i++){
			String str="";
			Row row = sheet2.createRow(i);
			Cell cell1 = row.createCell(0);
			cell1.setCellValue(boundary[i].x);
			Cell cell2 = row.createCell(1);
			cell2.setCellValue(boundary[i].y);
			Point3D_F64 p3= conversion2d.convertTo3D(boundary[i].x, boundary[i].y, conversion2d.center);
			str=p3.x+" "+p3.y+" "+p3.z;
			out1.println(str);
			
				
		}
		out1.close();
		try
        {
            //Write the workbook in file system
            FileOutputStream out = new FileOutputStream(new File("polygon_data.xls"));
            workbook2.write(out);
            out.close();
            System.out.println("polygon_data.xls written successfully on disk.");
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
	}

	

	
	
}
