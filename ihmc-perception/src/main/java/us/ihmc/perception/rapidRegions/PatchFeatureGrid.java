package us.ihmc.perception.rapidRegions;

import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencl.OpenCLManager;

public class PatchFeatureGrid
{
   private OpenCLManager openCLManager;

   private int rows = 0;
   private int columns = 0;

   private BytedecoImage nxImage;
   private BytedecoImage nyImage;
   private BytedecoImage nzImage;
   private BytedecoImage cxImage;
   private BytedecoImage cyImage;
   private BytedecoImage czImage;

   public PatchFeatureGrid(OpenCLManager openCLManager, int columns, int rows)
   {
      this.openCLManager = openCLManager;
      this.columns = columns;
      this.rows = rows;

      nxImage = new BytedecoImage(columns, rows, opencv_core.CV_32FC1);
      nyImage = new BytedecoImage(columns, rows, opencv_core.CV_32FC1);
      nzImage = new BytedecoImage(columns, rows, opencv_core.CV_32FC1);
      cxImage = new BytedecoImage(columns, rows, opencv_core.CV_32FC1);
      cyImage = new BytedecoImage(columns, rows, opencv_core.CV_32FC1);
      czImage = new BytedecoImage(columns, rows, opencv_core.CV_32FC1);
   }

   public void resize(int columns, int rows)
   {
      this.columns = columns;
      this.rows = rows;

      nxImage.resize(columns, rows, openCLManager, null);
      nyImage.resize(columns, rows, openCLManager, null);
      nzImage.resize(columns, rows, openCLManager, null);
      cxImage.resize(columns, rows, openCLManager, null);
      cyImage.resize(columns, rows, openCLManager, null);
      czImage.resize(columns, rows, openCLManager, null);
   }

   public void createOpenCLImages()
   {
      nxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      nyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      nzImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      cxImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      cyImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      czImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void readOpenCLImages()
   {
      nxImage.readOpenCLImage(openCLManager);
      nyImage.readOpenCLImage(openCLManager);
      nzImage.readOpenCLImage(openCLManager);
      cxImage.readOpenCLImage(openCLManager);
      cyImage.readOpenCLImage(openCLManager);
      czImage.readOpenCLImage(openCLManager);
   }

   public void destroy()
   {
      nxImage.destroy(openCLManager);
      nyImage.destroy(openCLManager);
      nzImage.destroy(openCLManager);
      cxImage.destroy(openCLManager);
      cyImage.destroy(openCLManager);
      czImage.destroy(openCLManager);
   }

   public void getNormal(int i, Vector3D vectorToPack)
   {
      getNormal(i / columns, i % columns, vectorToPack);
   }

   public void getCentroid(int i, Point3D pointToPack)
   {
      getCentroid(i / columns, i % columns, pointToPack);
   }

   public void getNormal(int row, int col, Vector3D vectorToPack)
   {
      vectorToPack.set(nxImage.getFloat(row, col), nyImage.getFloat(row, col), nzImage.getFloat(row, col));
   }

   public void getCentroid(int row, int col, Point3D pointToPack)
   {
      pointToPack.set(cxImage.getFloat(row, col), cyImage.getFloat(row, col), czImage.getFloat(row, col));
   }

   public int getRows()
   {
      return rows;
   }

   public int getColumns()
   {
      return columns;
   }

   public int getTotal()
   {
      return rows * columns;
   }

   public BytedecoImage getNxImage()
   {
      return nxImage;
   }

   public BytedecoImage getNyImage()
   {
      return nyImage;
   }

   public BytedecoImage getNzImage()
   {
      return nzImage;
   }

   public BytedecoImage getCxImage()
   {
      return cxImage;
   }

   public BytedecoImage getCyImage()
   {
      return cyImage;
   }

   public BytedecoImage getCzImage()
   {
      return czImage;
   }
}
