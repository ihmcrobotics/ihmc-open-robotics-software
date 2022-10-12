package us.ihmc.bytedeco.slamWrapper;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Vector4D;

public class FactorGraph
{
   private SlamWrapper.FactorGraphExternal factorGraphExternal;

   public FactorGraph()
   {
      factorGraphExternal = new SlamWrapper.FactorGraphExternal();
   }

   public SlamWrapper.FactorGraphExternal getFactorGraphExternal()
   {
      return factorGraphExternal;
   }

   public void addPriorPoseFactor(int index, Pose3D pose)
   {
      factorGraphExternal.addPriorPoseFactor(index,
                                             new float[] {(float) pose.getYaw(),
                                                          (float) pose.getPitch(),
                                                          (float) pose.getRoll(),
                                                          (float) pose.getX(),
                                                          (float) pose.getY(),
                                                          (float) pose.getZ()});
   }

   public void addOdometryFactor(Pose3D odometry, int poseId)
   {
      factorGraphExternal.addOdometryFactor(new float[] {(float) odometry.getYaw(),
                                                         (float) odometry.getPitch(),
                                                         (float) odometry.getRoll(),
                                                         (float) odometry.getX(),
                                                         (float) odometry.getY(),
                                                         (float) odometry.getZ()}, poseId);
   }

   public void addOrientedPlaneFactor(Vector4D plane, int lmId, int poseIndex)
   {
      factorGraphExternal.addOrientedPlaneFactor(new float[] {plane.getX32(), plane.getY32(), plane.getZ32(), plane.getS32()}, lmId, poseIndex);
   }

   public void optimize()
   {
      factorGraphExternal.optimize();
   }

   public void optimizeISAM2(int numberOfUpdates)
   {
      factorGraphExternal.optimizeISAM2((byte) numberOfUpdates);
   }

   public void clearISAM2()
   {
      factorGraphExternal.clearISAM2();
   }

   public void setPoseInitialValue(int index, Pose3D pose)
   {
      factorGraphExternal.setPoseInitialValue(index,
                                              new float[] {(float) pose.getYaw(),
                                                           (float) pose.getPitch(),
                                                           (float) pose.getRoll(),
                                                           (float) pose.getX(),
                                                           (float) pose.getY(),
                                                           (float) pose.getZ()});
   }

   public void setOrientedPlaneInitialValue(int landmarkId, Vector4D plane)
   {
      factorGraphExternal.setOrientedPlaneInitialValue(landmarkId, new float[] {plane.getX32(), plane.getY32(), plane.getZ32(), plane.getS32()});
   }

   public void createOdometryNoiseModel(float[] odomVariance)
   {
      factorGraphExternal.createOdometryNoiseModel(odomVariance);
   }

   public void createOrientedPlaneNoiseModel(float[] lmVariances)
   {
      factorGraphExternal.createOrientedPlaneNoiseModel(lmVariances);
   }

   public void printResults()
   {
      factorGraphExternal.printResults();
   }
}
