package us.ihmc.avatar.reachabilityMap.visualizer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape;
import us.ihmc.avatar.reachabilityMap.voxelPrimitiveShapes.SphereVoxelShape.SphereVoxelType;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.ModifiableMeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ReachabilitySphereMapVisualizers
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static void visualizeSphereVoxelShape()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.startOnAThread();
      Point3D sphereOrigin = new Point3D(0.0, 0.0, 1.0);
      scs.setCameraFix(sphereOrigin.getX(), sphereOrigin.getY(), sphereOrigin.getZ());
      double voxelSize = 0.10;
      int numberOfRays = 20;
      int numberOfRotationsAroundRay = 10;
      SphereVoxelShape sphereVoxelShape = new SphereVoxelShape(worldFrame, voxelSize, numberOfRays, numberOfRotationsAroundRay,
            SphereVoxelType.graspAroundSphere);

      for (int i = 0; i < sphereVoxelShape.getNumberOfRays(); i++)
      {
         for (int j = 0; j < sphereVoxelShape.getNumberOfRotationsAroundRay(); j++)
         {
            FrameVector3D translationFromVoxelOrigin = new FrameVector3D();
            FrameQuaternion orientation = new FrameQuaternion();
            sphereVoxelShape.getPose(translationFromVoxelOrigin, orientation, i, j);

            Graphics3DObject staticLinkGraphics = new Graphics3DObject();
            staticLinkGraphics.translate(sphereOrigin);
            staticLinkGraphics.translate(translationFromVoxelOrigin);
            RotationMatrix rotationMatrix = new RotationMatrix();
            rotationMatrix.set(orientation);
            staticLinkGraphics.rotate(rotationMatrix);
            staticLinkGraphics.addCoordinateSystem(0.05);
            scs.addStaticLinkGraphics(staticLinkGraphics);
         }
      }
   }

   public static void visualizePointsOnSphereUsingSpiralBasedAlgorithm()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.startOnAThread();
      Point3D sphereOrigin = new Point3D(0.0, 0.0, 1.0);

      double sphereRadius = 0.10;
      int numberOfPointsToGenerate = 200;
      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(sphereOrigin, sphereRadius, numberOfPointsToGenerate);

      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.translate(sphereOrigin);
      AppearanceDefinition red = YoAppearance.Red();
      red.setTransparency(0.7);
      staticLinkGraphics.addSphere(sphereRadius, red);
      scs.addStaticLinkGraphics(staticLinkGraphics);

      for (int i = 0; i < numberOfPointsToGenerate; i++)
      {
         staticLinkGraphics = new Graphics3DObject();
         staticLinkGraphics.translate(pointsOnSphere[i]);
         staticLinkGraphics.addSphere(0.005);
         scs.addStaticLinkGraphics(staticLinkGraphics);
      }
   }

   public static void visualizeSVDWithPerfectLine()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.startOnAThread();

      // Draw points along a perfect line
      Point3D origin = new Point3D(0.0, 0.0, 1.0);
      Vector3D direction = new Vector3D(0.5, 0.2, 0.7);

      List<Point3D> listOfPoints = new ArrayList<>();

      int numberOfPoints = 300;

      for (int i = 0; i < numberOfPoints; i++)
      {
         double alpha = (double) i / (double) numberOfPoints;
         Point3D point = new Point3D();
         point.set(direction);
         point.scale(alpha);
         point.add(origin);
         listOfPoints.add(point);

         Graphics3DObject pointViz = new Graphics3DObject();
         pointViz.translate(point);
         pointViz.addSphere(0.01);
         scs.addStaticLinkGraphics(pointViz);
      }

      final RotationMatrix rotationMatrix3d = new RotationMatrix();
      Point3D average = new Point3D();
      Vector3D principalValues = new Vector3D();

      PrincipalComponentAnalysis3D principalComponentAnalysis3D = new PrincipalComponentAnalysis3D();
      principalComponentAnalysis3D.setPointCloud(listOfPoints);
      principalComponentAnalysis3D.compute();
      principalComponentAnalysis3D.getPrincipalFrameRotationMatrix(rotationMatrix3d);
      principalComponentAnalysis3D.getVariance(principalValues);
      principalComponentAnalysis3D.getMean(average);

      Graphics3DObject pcaFrame = new Graphics3DObject();
      pcaFrame.translate(average);
      pcaFrame.rotate(rotationMatrix3d);
      pcaFrame.addCoordinateSystem(0.5);
      scs.addStaticLinkGraphics(pcaFrame);
   }

   public static void visualizeSVDWithGaussianPointCloud()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.startOnAThread();

      // Draw points along a perfect line
      Random random = new Random(34L);
      Point3D origin = new Point3D(0.0, 0.0, 1.0);

      List<Point3D> listOfPoints = new ArrayList<>();

      int numberOfPoints = 10000;
      double radiusX = 0.1;
      double radiusY = 0.3;
      double radiusZ = 0.01;
      RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      ModifiableMeshDataHolder modifiableMeshDataHolder = new ModifiableMeshDataHolder();

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D point = new Point3D();
         point.setX(radiusX * random.nextGaussian());
         point.setY(radiusY * random.nextGaussian());
         point.setZ(radiusZ * random.nextGaussian());
         point.add(origin);
         transform.transform(point);
         listOfPoints.add(point);

         MeshDataHolder tetrahedron = MeshDataGenerator.Tetrahedron(0.01);
         translateMeshVertices(tetrahedron, point);
         modifiableMeshDataHolder.add(tetrahedron, true);
      }

      Graphics3DObject pointViz = new Graphics3DObject();
      pointViz.addMeshData(modifiableMeshDataHolder.createMeshDataHolder(), YoAppearance.Black());
      scs.addStaticLinkGraphics(pointViz);

      final RotationMatrix rotationMatrix3d = new RotationMatrix();
      Point3D average = new Point3D();
      Vector3D principalValues = new Vector3D();

      PrincipalComponentAnalysis3D principalComponentAnalysis3D = new PrincipalComponentAnalysis3D();
      principalComponentAnalysis3D.setPointCloud(listOfPoints);
      principalComponentAnalysis3D.compute();
      principalComponentAnalysis3D.getPrincipalFrameRotationMatrix(rotationMatrix3d);
      principalComponentAnalysis3D.getStandardDeviation(principalValues);
      principalComponentAnalysis3D.getMean(average);

      System.out.println("PrincipalValues: " + principalValues);

      Graphics3DObject pcaFrame = new Graphics3DObject();
      pcaFrame.translate(average);
      pcaFrame.rotate(rotationMatrix3d);
      pcaFrame.addCoordinateSystem(0.5);
      scs.addStaticLinkGraphics(pcaFrame);
   }

   private static void translateMeshVertices(MeshDataHolder meshToModify, Tuple3DReadOnly offset)
   {
      Point3D32 offset3f = new Point3D32(offset);

      for (Point3D32 vertex : meshToModify.getVertices())
         vertex.add(offset3f);
   }

   public static void testSVDWithSphereVoxel()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      scs.startOnAThread();

      final Point3D sphereOrigin = new Point3D(0.0, 0.0, 1.0);
      Graphics3DObject originViz = new Graphics3DObject();
      originViz.translate(sphereOrigin);
      originViz.addSphere(0.01, YoAppearance.AluminumMaterial());
      scs.addStaticLinkGraphics(originViz);

      double sphereRadius = 0.3;
      int numberOfPointsToGenerate = 1000;
      Point3D[] allPointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(sphereOrigin, sphereRadius, numberOfPointsToGenerate);
      List<Point3D> selectedPoints = new ArrayList<>();

      // Select only certain that we'll use to fit a cone shape to.
      for (Point3D point : allPointsOnSphere)
      {
         if (point.getX() >= 0.2)
            selectedPoints.add(point);
      }

      for (Point3D point : selectedPoints)
      {
         Graphics3DObject pointViz = new Graphics3DObject();
         pointViz.translate(point);
         pointViz.addSphere(0.01);
         scs.addStaticLinkGraphics(pointViz);
      }

      final RotationMatrix rotationMatrix = new RotationMatrix();
      final Point3D average = new Point3D();
      Vector3D principalValues = new Vector3D();

      PrincipalComponentAnalysis3D principalComponentAnalysis3D = new PrincipalComponentAnalysis3D();
      principalComponentAnalysis3D.setPointCloud(selectedPoints);
      principalComponentAnalysis3D.compute();
      principalComponentAnalysis3D.getPrincipalFrameRotationMatrix(rotationMatrix);
      principalComponentAnalysis3D.getStandardDeviation(principalValues);
      principalComponentAnalysis3D.getMean(average);

      Vector3D sphereOriginToAverage = new Vector3D();
      sphereOriginToAverage.sub(average, sphereOrigin);
      Vector3D thirdAxis = new Vector3D();
      rotationMatrix.getColumn(2, thirdAxis);

      if (sphereOriginToAverage.dot(thirdAxis) < 0.0)
      {
         // Rotate the frame of PI around the principal axis, such that the third axis is pointing towards the point cloud.
         RotationMatrix invertThirdAxis = new RotationMatrix();
         invertThirdAxis.setToRollOrientation(Math.PI);
         rotationMatrix.multiply(invertThirdAxis);
      }

      // Transform the cone head to  the parent frame, in this case world.
      RigidBodyTransform coneTransform = new RigidBodyTransform();
      coneTransform.getRotation().set(rotationMatrix);
      coneTransform.getTranslation().set(sphereOrigin.getX(), sphereOrigin.getY(), sphereOrigin.getZ());

      // Build the cone
      double smallestDotProduct = Double.POSITIVE_INFINITY;
      rotationMatrix.getColumn(2, thirdAxis);
      Vector3D testedRay = new Vector3D();
      Vector3D mostOpenedRay = new Vector3D();

      // Find the point that is the farthest from the 
      for (Point3D point : selectedPoints)
      {
         testedRay.sub(point, sphereOrigin);
         double absDotProduct = Math.abs(testedRay.dot(thirdAxis));
         if (absDotProduct < smallestDotProduct)
         {
            smallestDotProduct = absDotProduct;
            mostOpenedRay.set(testedRay);
         }
      }

      Graphics3DObject coneFrameViz = new Graphics3DObject();
      coneFrameViz.transform(coneTransform);
      coneFrameViz.addCoordinateSystem(0.5);
      scs.addStaticLinkGraphics(coneFrameViz);

      double coneBaseRadius = Math.sqrt(principalValues.getX() * principalValues.getX() + principalValues.getY() * principalValues.getY());//radiusVector.length();
      double coneHeight = mostOpenedRay.dot(thirdAxis);

      Graphics3DObject coneGraphic = new Graphics3DObject();
      coneGraphic.transform(coneTransform);
      coneGraphic.translate(0.0, 0.0, coneHeight);
      coneGraphic.rotate(Math.PI, Axis3D.Y);
      coneGraphic.addCone(coneHeight, coneBaseRadius, YoAppearance.DarkGreen());
      scs.addStaticLinkGraphics(coneGraphic);
   }

   public static void main(String[] args)
   {
      //      visualizeSVDWithPerfectLine();
      visualizeSVDWithGaussianPointCloud();
      //      testSVDWithSphereVoxel();
      //            visualizeSphereVoxelShape();
      //            visualizePointsOnSphereUsingSpiralBasedAlgorithm();
   }
}
