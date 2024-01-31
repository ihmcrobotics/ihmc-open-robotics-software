package us.ihmc.perception.opencl;

import org.bytedeco.opencl._cl_mem;

public class OpenCLIntMemory
{
   private long numberOfInts;
   private _cl_mem openCLBufferObject;

   public OpenCLIntMemory(int numberOfFloats)
   {
      resize(numberOfFloats, null);
   }

   public void destroy(OpenCLManager openCLManager)
   {
      if (openCLBufferObject != null)
      {
         openCLManager.releaseBufferObject(openCLBufferObject);
         openCLBufferObject.releaseReference();
         openCLBufferObject = null;
      }
   }

   public void resize(int numberOfFloats,  OpenCLManager openCLManager)
   {
      if (numberOfFloats == this.numberOfInts)
         return;

      this.numberOfInts = numberOfFloats;

      boolean openCLObjectCreated = openCLBufferObject != null;
      destroy(openCLManager);

      if (openCLObjectCreated)
      {
         createOpenCLBufferObject(openCLManager);
      }
   }

   public void createOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLBufferObject = openCLManager.createBufferObject(numberOfInts * Integer.BYTES, null);
   }

   public _cl_mem getOpenCLBufferObject()
   {
      return openCLBufferObject;
   }
}
