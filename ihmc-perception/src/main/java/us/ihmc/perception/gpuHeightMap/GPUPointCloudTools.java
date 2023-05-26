package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import sensor_msgs.msg.dds.PointCloud;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLManager;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

public class GPUPointCloudTools
{
   private final static int defaultMaxNumberOfPoints = 10000;

   private final int maxNumberOfPoints;
   private final OpenCLManager openCLManager = new OpenCLManager();

   private OpenCLFloatBuffer inputPointCloudBuffer;
   private OpenCLFloatBuffer transformedPointCloudBuffer;
   private final OpenCLFloatBuffer transformBuffer = new OpenCLFloatBuffer(12);
   private final _cl_program heightMapToolsProgram;
   private final _cl_kernel transformPointsKernel;

   public GPUPointCloudTools()
   {
      this(defaultMaxNumberOfPoints);
   }

   public GPUPointCloudTools(int maxNumberOfPoints)
   {
      this.maxNumberOfPoints = maxNumberOfPoints;

//      inputPointCloudBuffer = new OpenCLFloatBuffer(maxNumberOfPoints * 3 * Float.BYTES);
//      transformedPointCloudBuffer = new OpenCLFloatBuffer(maxNumberOfPoints * 3 * Float.BYTES);
//
//      inputPointCloudBuffer.createOpenCLBufferObject(openCLManager);
//      transformedPointCloudBuffer.createOpenCLBufferObject(openCLManager);

      transformBuffer.createOpenCLBufferObject(openCLManager);
      heightMapToolsProgram = openCLManager.loadProgram("GPUPointCloudTools");
      transformPointsKernel = openCLManager.createKernel(heightMapToolsProgram, "transformPointsKernel");
   }

   // point cloud is expected to be in sensor frame
   public void transformPoints(PointCloud pointCloud, ReferenceFrame pointCloudFrame)
   {
      transformPoints(pointCloud.getPoints(), pointCloudFrame);
   }

   public List<Point3D> transformPoints(List<Point3D32> points, ReferenceFrame pointCloudFrame)
   {
      return transformPoints(points, pointCloudFrame.getTransformToWorldFrame());
   }

   public List<Point3D> transformPoints(List<Point3D32> points, RigidBodyTransformReadOnly transformToDesiredFrame)
   {
      inputPointCloudBuffer = new OpenCLFloatBuffer(points.size() * 3);
      transformedPointCloudBuffer = new OpenCLFloatBuffer(points.size() * 3);

      inputPointCloudBuffer.createOpenCLBufferObject(openCLManager);
      transformedPointCloudBuffer.createOpenCLBufferObject(openCLManager);

      packPointCloudIntoFloatBUffer(points, inputPointCloudBuffer.getBackingDirectFloatBuffer());
      inputPointCloudBuffer.writeOpenCLBufferObject(openCLManager);

      populateTransformBuffer(transformToDesiredFrame);

      openCLManager.setKernelArgument(transformPointsKernel, 0, inputPointCloudBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(transformPointsKernel, 1, transformBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(transformPointsKernel, 2, transformedPointCloudBuffer.getOpenCLBufferObject());

      openCLManager.execute1D(transformPointsKernel, points.size());

      transformedPointCloudBuffer.readOpenCLBufferObject(openCLManager);

      return retrieveTransformedPoints(points.size(), transformedPointCloudBuffer.getBackingDirectFloatBuffer());
   }

   private void packPointCloudIntoFloatBUffer(List<Point3D32> points, FloatBuffer floatBufferToPack)
   {
      int index = 0;
      for (int i = 0; i < points.size(); i++)
      {
         floatBufferToPack.put(index++, points.get(i).getX32());
         floatBufferToPack.put(index++, points.get(i).getY32());
         floatBufferToPack.put(index++, points.get(i).getZ32());
      }
   }

   private List<Point3D> retrieveTransformedPoints(int numberOfPoints, FloatBuffer transformedPointsBuffer)
   {
      List<Point3D> transformedPoints = new ArrayList<>();
      int index = 0;
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D point = new Point3D();
         point.setX(transformedPointsBuffer.get(index++));
         point.setY(transformedPointsBuffer.get(index++));
         point.setZ(transformedPointsBuffer.get(index++));
         transformedPoints.add(point);
      }

      return transformedPoints;
   }

   private final RotationMatrixBasics rotation = new RotationMatrix();

   private void populateTransformBuffer(RigidBodyTransformReadOnly desiredTransform)
   {
      rotation.set(desiredTransform.getRotation());

      int index = 0;
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
            transformBuffer.getBytedecoFloatBufferPointer().put(index++, (float) rotation.getElement(i, j));
      }
      for (int i = 0; i < 3; i++)
         transformBuffer.getBytedecoFloatBufferPointer().put(index++, (float) desiredTransform.getTranslation().getElement(i));

      transformBuffer.writeOpenCLBufferObject(openCLManager);
   }

}
