package us.ihmc.graphics3DAdapter;

import javax.media.j3d.Transform3D;


public interface GPULidar
{
// protected static final double NINETY_DEG = Math.PI / 2;
//
// protected GPULidarCallback callback;
// protected double fieldOfView;
// private float[][] frameSweeps;
// private int scanIndex = -1;
// private int framesPerSweep;
// private int scansPerSweep;
// protected int scansPerFrame;
// private double sweepStartAngle;
// private Transform3D userTransform;
//
// protected GPULidar(GPULidarCallback callback, double fieldOfView, int scansPerSweep)
// {
//    this.callback = callback;
//    this.fieldOfView = fieldOfView;
//    this.scansPerSweep = scansPerSweep;
//
//    init();
// }
//
// private void init()
// {
//    framesPerSweep = (int) Math.ceil(fieldOfView / NINETY_DEG);
//    sweepStartAngle = -Math.PI / 4.0 * (framesPerSweep - 1);
//    scansPerFrame = (int) (scansPerSweep * ((framesPerSweep * NINETY_DEG) / fieldOfView));
//
//    System.out.println("Frames per sweep: " + framesPerSweep);
//    System.out.println("Sweep start angle: " + sweepStartAngle);
//    System.out.println("Scans per frame: " + scansPerFrame);
//
//    frameSweeps = new float[framesPerSweep][scansPerFrame];
// }
//
// @Override
// public void scan(float[] frameSweep, Transform3D currentTransform, double time, GPULidar gpuLidar)
// {
////   try
////   {
////      Thread.sleep(300);
////   }
////   catch (InterruptedException e)
////   {
////      // TODO Auto-generated catch block
////      e.printStackTrace();
////   }
//    callback.scan(frameSweep, currentTransform, time, gpuLidar);
//
//    return;
//
////   System.out.println(scanIndex);
////   if (scanIndex < 0)
////   {
////      userTransform = new Transform3D(currentTransform);
////      setTransformFromWorld(rotateLocal(userTransform, sweepStartAngle, Axis.Z), time);
////   }
////   else
////   {
////      frameSweeps[scanIndex] = frameSweep;
////
////      if (scanIndex >= framesPerSweep - 1)
////      {
////         setTransformFromWorld(userTransform, time);
////         callback.scan(assembleScans(), userTransform, time, gpuLidar);
////
////         scanIndex = -1;
////
////         return;
////      }
////      else
////      {
////         setTransformFromWorld(rotateLocal(userTransform, -sweepStartAngle + NINETY_DEG * scanIndex, Axis.Z), time);
////      }
////   }
////   scanIndex++;
// }
//
// public Transform3D rotateLocal(Transform3D original, double angle, Axis axis)
// {
//    Transform3D rotated = new Transform3D();
//    Transform3D rotationIncrement = new Transform3D();
//
//    if (axis == Axis.X)
//    {
//       rotationIncrement.rotX(angle);
//    }
//    else if (axis == Axis.Y)
//    {
//       rotationIncrement.rotY(angle);
//    }
//    else if (axis == Axis.Z)
//    {
//       rotationIncrement.rotZ(angle);
//    }
//
//    rotated.mul(original, rotationIncrement);
//
//    return rotated;
// }
//
// public float[] assembleScans()
// {
//    float[] fullSweep = Floats.concat(frameSweeps);
//
//    float[] shortenedSweep = new float[scansPerSweep];
//
//    int startIndex = (fullSweep.length - scansPerSweep) / 2;
//
//    for (int i = 0; i < scansPerSweep; i++)
//    {
//       shortenedSweep[i] = fullSweep[i + startIndex];
//    }
//
//    return shortenedSweep;
// }

   public abstract void setTransformFromWorld(Transform3D transformToWorld, double time);
}
