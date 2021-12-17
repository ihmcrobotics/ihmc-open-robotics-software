package us.ihmc.gdx.perception;

import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Loader;
import org.bytedeco.opencl._cl_mem;
import us.ihmc.perception.OpenCLManager;

public class GDXGPUPlanarRegionExtraction
{
   private final ImFloat mergeDistanceThreshold = new ImFloat(0.016f);
   private final ImFloat mergeAngularThreshold = new ImFloat(0.82f);
   private final ImFloat filterDisparityThreshold = new ImFloat(2000);
   private final ImInt inputHeight = new ImInt(0);
   private final ImInt inputWidth = new ImInt(0);
   private final ImInt kernelSliderLevel = new ImInt(2);
   private final ImInt filterKernelSize = new ImInt(4);
   private final ImFloat depthFx = new ImFloat(0);
   private final ImFloat depthFy = new ImFloat(0);
   private final ImFloat depthCx = new ImFloat(0);
   private final ImFloat depthCy = new ImFloat(0);
   private final OpenCLManager openCLManager = new OpenCLManager();
   private int numberOfFloatParameters = 16;
   private _cl_mem parametersBufferObject;
   private long parametersBufferSizeInBytes;
   private FloatPointer parametersNativeCPUPointer;

   public GDXGPUPlanarRegionExtraction()
   {

   }

   public void create()
   {
      openCLManager.create();

      parametersBufferSizeInBytes = numberOfFloatParameters * Loader.sizeof(FloatPointer.class);
      parametersBufferObject = openCLManager.createBufferObject(parametersBufferSizeInBytes);
      parametersNativeCPUPointer = new FloatPointer(numberOfFloatParameters);
   }

   private void generateRegionsFromDepth()
   {
      // depth image
      // timestamp


   }

   private void generatePatchGraph()
   {

   }

   private void uploadParametersToGPU()
   {
      int patchHeight = kernelSliderLevel.get();
      int patchWidth = kernelSliderLevel.get();
      int subHeight = inputHeight.get() / patchHeight;
      int subWidth = inputWidth.get() / patchWidth;
      int filterSubHeight = inputHeight.get() / filterKernelSize.get();
      int filterSubWidth = inputWidth.get() / filterKernelSize.get();

      FloatPointer parameters = new FloatPointer(numberOfFloatParameters);
      parameters.put(0, filterDisparityThreshold.get());
      parameters.put(1, mergeAngularThreshold.get());
      parameters.put(2, mergeDistanceThreshold.get());
      parameters.put(3, patchHeight);
      parameters.put(4, patchWidth);
      parameters.put(5, subHeight);
      parameters.put(6, subWidth);
      parameters.put(7, depthFx.get());
      parameters.put(8, depthFy.get());
      parameters.put(9, depthCx.get());
      parameters.put(10, depthCy.get());
      parameters.put(11, filterKernelSize.get());
      parameters.put(12, filterSubHeight);
      parameters.put(13, filterSubWidth);
      parameters.put(14, inputHeight.get());
      parameters.put(15, inputWidth.get());

      openCLManager.enqueueWriteBuffer(parametersBufferObject, parametersBufferSizeInBytes, parameters);
   }
}
