package us.ihmc.manipulation.planning.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.transformables.Pose;

public class SquareFittingFactory
{
   private boolean DEBUG = false;
   private PlanarRegion planarRegion;
   private ArrayList<Point3D32> vertices = new ArrayList<Point3D32>();
   private SolarPanel solarPanel;
   
   private Vector3D normalVector;
   private RotationMatrix squareRotationMatrix;
   private Point3D squarePosition;
   
   private double squareSizeX;
   private double squareSizeY;
   
   private LineEquation lineOneAxisX = new LineEquation();
   private LineEquation lineTwoAxisX = new LineEquation();
   
   private LineEquation lineOneAxisY = new LineEquation();
   private LineEquation lineTwoAxisY = new LineEquation();
   
   Point3D squarePoint1 = new Point3D();
   Point3D squarePoint2 = new Point3D();
   Point3D squarePoint3 = new Point3D();
   Point3D squarePoint4 = new Point3D();
   
   class LineEquation
   {
      public Point3D point;
      public Vector3D vector;
      
      LineEquation()
      {
         this.point = new Point3D();
         this.vector = new Vector3D();
      }
      
      LineEquation(Tuple3DBasics point, Vector3D vector)
      {
         this.point = new Point3D(point);
         this.vector = vector;
      }
      
      void setPoint(Tuple3DBasics point)
      {
         this.point = new Point3D(point);
      }
      
      void setVector(Vector3D vector)
      {
         this.vector = vector;
      }
      
      Point3D getPointOnLine(double t)
      {
         return new Point3D(point.getX() + t*vector.getX(), point.getY() + t*vector.getY(), point.getZ() + t*vector.getZ());
      }
      
      Point3D getIntersectedPoint(LineEquation otherLine)
      {         
         double x1 = this.point.getX();
         double y1 = this.point.getY();
         double z1 = this.point.getZ();
         double a1 = this.vector.getX();
         double b1 = this.vector.getY();
         double c1 = this.vector.getZ();
         
         double x2 = otherLine.point.getX();
         double y2 = otherLine.point.getY();
         double z2 = otherLine.point.getZ();
         double a2 = otherLine.vector.getX();
         double b2 = otherLine.vector.getY();
         double c2 = otherLine.vector.getZ();
         
         double tOfOtherLine = (a1*y2-a1*y1-b1*x2+b1*x1)/(a2*b1 - a1*b2);
                  
         return otherLine.getPointOnLine(tOfOtherLine);
      }
   }   
   
   public SquareFittingFactory()
   {
      
   }
   
   public SquareFittingFactory(PlanarRegion planarRegion)
   {
      setPlanarRegion(planarRegion);
   }
   
   public SquareFittingFactory(Vector3D normalToPack, ArrayList<Point3D32> vertices)
   {
      this.vertices = vertices; 
      this.normalVector = normalToPack;
      updateSquare();
   }
   
   public void setPlanarRegion(PlanarRegion planarRegion)
   {
      this.planarRegion = planarRegion;
      updateVertices();      
      updateNormalVector();
      updateSquare();
   }
   
   private void updateSquare()
   {
      fittingSquare();
      
      updateLineEquationsOfSquare();
      
      updateCenterPosition();
      /*
       * here is for converting as we want.
       */      
      updateSolarPanel();  
      
   }
   
   private void updateVertices()
   {
      vertices.clear();
      
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);
      
      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         MeshDataHolder polygon = MeshDataGenerator.Polygon(transformToWorld, convexPolygon);
         
         if(DEBUG)
            PrintTools.info("polygonIndex "+polygonIndex);
         if(polygon != null)
         {
            if(DEBUG)
               PrintTools.info("Vectices "+polygon.getVertices().length);
            
            for(int i=0;i<polygon.getVertices().length;i++)
            {
               if(DEBUG)
                  PrintTools.info(""+i+" "+polygon.getVertices()[i].getX()+" "+polygon.getVertices()[i].getY()+" "+polygon.getVertices()[i].getZ());
               vertices.add(polygon.getVertices()[i]);
            }
         }         
      }
      
      if(DEBUG)
         PrintTools.info("Number Of points are "+ vertices.size());
   }
   

   private void updateNormalVector()
   {      
      this.normalVector = new Vector3D();
      this.planarRegion.getNormal(normalVector); 
   }
         
   private void fittingSquare()
   {
      for(int i=0;i<vertices.size();i++)
      {
         PrintTools.info(" "+i+" "+vertices.get(i).getX()+" "+vertices.get(i).getY()+" "+vertices.get(i).getZ());
      }
      
      double appendingPitchDirection = Math.acos(normalVector.getZ());
      if(normalVector.getX() < 0)
         appendingPitchDirection = -appendingPitchDirection;
      double appendingYawDirection = Math.asin(normalVector.getY()/Math.sin(appendingPitchDirection));
      
      if(DEBUG)
         PrintTools.info("Currently appendingYawDirection  " + appendingYawDirection);
      if(DEBUG)
         PrintTools.info("Currently appendingPitchDirection " + appendingPitchDirection);
           
      int numberOfSampling = 30;
      double minRangeOfSampling = Math.PI * (-44.0/180.0);
      double maxRangeOfSampling = Math.PI * (44.0/180.0);
      double minArea = Double.MAX_VALUE;
      for(int i=0;i<numberOfSampling;i++)
      {
         double appendingYawAngle = (maxRangeOfSampling - minRangeOfSampling) * i/(numberOfSampling-1) + minRangeOfSampling;
         if(DEBUG)
            PrintTools.info(""+i+" "+ appendingYawAngle);
         
         RotationMatrix perturbedRotationMatrix = new RotationMatrix();
         perturbedRotationMatrix.appendYawRotation(appendingYawDirection);
         perturbedRotationMatrix.appendPitchRotation(appendingPitchDirection);
         perturbedRotationMatrix.appendYawRotation(appendingYawAngle);
         
         if(minArea > getAreaUnderRotationMatrix(perturbedRotationMatrix))
         {
            minArea = getAreaUnderRotationMatrix(perturbedRotationMatrix);
            squareRotationMatrix = perturbedRotationMatrix;
            if(DEBUG)
               PrintTools.info("appendingYawAngle "+appendingYawAngle);
         }
      }      
      
      Vector3D principalAxisX = getPrincipalAxisX(squareRotationMatrix);
      Vector3D principalAxisY = getPrincipalAxisY(squareRotationMatrix);
      
      squareSizeX = getSizeAlongAxis(principalAxisX);
      squareSizeY = getSizeAlongAxis(principalAxisY);
   }
   
   private void updateLineEquationsOfSquare()
   {
      Vector3D principalAxisX = getPrincipalAxisX(squareRotationMatrix);
      Vector3D principalAxisY = getPrincipalAxisY(squareRotationMatrix);
      
      
      System.out.println(" principalAxisX "+principalAxisX);
      System.out.println(" principalAxisY "+principalAxisY);
      
      double distance;
      
      distance = -Double.MAX_VALUE;      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance < getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisX))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisX);
            lineOneAxisX = new LineEquation(vertices.get(i), principalAxisX);
            squarePoint1 = new Point3D(vertices.get(i));
         }
      }
      
      distance = -Double.MAX_VALUE;      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance < getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisY))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisY);
            lineOneAxisY = new LineEquation(vertices.get(i), principalAxisY);
            squarePoint2 = new Point3D(vertices.get(i));
         }
      }
      
      distance = Double.MAX_VALUE;      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance > getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisX))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisX);
            lineTwoAxisX = new LineEquation(vertices.get(i), principalAxisX);
            squarePoint3 = new Point3D(vertices.get(i));
         }
      }
      
      distance = Double.MAX_VALUE;      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance > getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisY))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisY);
            lineTwoAxisY = new LineEquation(vertices.get(i), principalAxisY);  
            squarePoint4 = new Point3D(vertices.get(i));
         }
      }
   }
   
   private void updateCenterPosition()
   {      
      Point3D pointOne = lineOneAxisX.getIntersectedPoint(lineOneAxisY);
      Point3D pointTwo = lineTwoAxisX.getIntersectedPoint(lineTwoAxisY);
      
      Point3D pointThree = lineOneAxisX.getIntersectedPoint(lineTwoAxisY);
      Point3D pointFour  = lineTwoAxisX.getIntersectedPoint(lineOneAxisY);
      
      Point3D centerOne = new Point3D((pointOne.getX()+pointTwo.getX())/2, (pointOne.getY()+pointTwo.getY())/2, (pointOne.getZ()+pointTwo.getZ())/2);
      Point3D centerTwo = new Point3D((pointThree.getX()+pointFour.getX())/2, (pointThree.getY()+pointFour.getY())/2, (pointThree.getZ()+pointFour.getZ())/2);
      
      System.out.println(squarePoint1);
      System.out.println(squarePoint2);
      System.out.println(squarePoint3);
      System.out.println(squarePoint4);
      
      System.out.println(pointOne);
      System.out.println(pointTwo);
      System.out.println(pointThree);
      System.out.println(pointFour);
      
      System.out.println(centerOne);
      System.out.println(centerTwo);
      
      squarePosition = new Point3D((centerOne.getX()+centerTwo.getX())/2, (centerOne.getY()+centerTwo.getY())/2, (centerOne.getZ()+centerTwo.getZ())/2);
      
      if(DEBUG)
         PrintTools.info("squarePosition "+ squarePosition.getX() +" "+ squarePosition.getY() +" "+squarePosition.getZ());
   }
   
   private void updateSolarPanel()
   {
      Quaternion solarPanelOrientation = new Quaternion(squareRotationMatrix);
      Pose solarPanelPose = new Pose(squarePosition, solarPanelOrientation);
      PrintTools.info("Fitted panel info ");
      System.out.println(solarPanelPose);
      System.out.println(squareSizeX);
      System.out.println(squareSizeY);
      solarPanel = new SolarPanel(solarPanelPose, squareSizeX, squareSizeY);
   }
   
   
   
   
   
   
   
   
   
   private Vector3D getPrincipalAxisX(RotationMatrix rotationMatrix)
   {
      return new Vector3D(rotationMatrix.getM00(), rotationMatrix.getM10(), rotationMatrix.getM20());
   }
   
   private Vector3D getPrincipalAxisY(RotationMatrix rotationMatrix)
   {
      return new Vector3D(rotationMatrix.getM01(), rotationMatrix.getM11(), rotationMatrix.getM21());
   }
   
   private double getAreaUnderRotationMatrix(RotationMatrix rotationMatrix)
   {
      Vector3D principalAxisX = getPrincipalAxisX(rotationMatrix);
      Vector3D principalAxisY = getPrincipalAxisY(rotationMatrix);
            
      return getSizeAlongAxis(principalAxisX)*getSizeAlongAxis(principalAxisY);
   }

   private double getSizeAlongAxis(Vector3D axis)
   {  
      return Math.abs(getMaximumDistanceFromPlane(new Point3D(), axis) - getMinimumDistanceFromPlane(new Point3D(), axis));
   }
   
   private double getMaximumDistanceFromPlane(Point3D planeCenter, Vector3D planeNormal)
   {
      double distance = 0.0;
      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance < getSingedDistancePointAndPlane(vertices.get(i), planeCenter, planeNormal))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), planeCenter, planeNormal);
         }
      }
      
      return distance;
   }
   
   private double getMinimumDistanceFromPlane(Point3D planeCenter, Vector3D planeNormal)
   {
      double distance = Double.MAX_VALUE;
      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance > getSingedDistancePointAndPlane(vertices.get(i), planeCenter, planeNormal))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), planeCenter, planeNormal);
         }
      }
      
      return distance;
   }
   
   private double getSingedDistancePointAndPlane(Tuple3DBasics point3d32, Point3D planeCenter, Vector3D planeNormal)
   {
      double planeA = planeNormal.getX();
      double planeB = planeNormal.getY();
      double planeC = planeNormal.getZ();
      double planeD = -planeA*planeCenter.getX()-planeB*planeCenter.getY()-planeC*planeCenter.getZ();

      double distance = (planeA*point3d32.getX() + planeB*point3d32.getY() + planeC*point3d32.getZ() + planeD)/(Math.sqrt(planeA*planeA + planeB*planeB + planeC*planeC));
      
      return distance;
   }
         
   public SolarPanel getSolarPanel()
   {
      return solarPanel;
   }
   
   
   
   
   
   
}
