package us.ihmc.perception.opencl;

import org.bytedeco.opencl._cl_mem;

public class OpenCLFloatMemory
{
   private long numberOfFloats;
   private _cl_mem openCLBufferObject;

   public OpenCLFloatMemory(int numberOfFloats)
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
      if (numberOfFloats == this.numberOfFloats)
         return;

      this.numberOfFloats = numberOfFloats;

      boolean openCLObjectCreated = openCLBufferObject != null;
      destroy(openCLManager);

      if (openCLObjectCreated)
      {
         createOpenCLBufferObject(openCLManager);
      }
   }

   public void createOpenCLBufferObject(OpenCLManager openCLManager)
   {
      openCLBufferObject = openCLManager.createBufferObject(numberOfFloats * Float.BYTES, null);
   }

   public _cl_mem getOpenCLBufferObject()
   {
      return openCLBufferObject;
   }
}
