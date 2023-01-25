package us.ihmc.perception.slamWrapper;

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

   public void addPriorPoseFactor(int index, float[] pose)
   {
      factorGraphExternal.addPriorPoseFactor(index, pose);
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

   public void addOdometryFactor(float[] odometry, int poseId)
   {
      factorGraphExternal.addOdometryFactor(odometry, poseId);
   }

   public void addOrientedPlaneFactor(Vector4D plane, int lmId, int poseIndex)
   {
      factorGraphExternal.addOrientedPlaneFactor(new float[] {plane.getX32(), plane.getY32(), plane.getZ32(), plane.getS32()}, lmId, poseIndex);
   }

   public void addOrientedPlaneFactor(float[] plane, int lmId, int poseIndex)
   {
      factorGraphExternal.addOrientedPlaneFactor(plane, lmId, poseIndex);
   }

   public void addOdometryFactorExtended(float[] plane, int lmId, int poseIndex)
   {
      factorGraphExternal.addOrientedPlaneFactor(plane, lmId, poseIndex);
   }

   public void setPoseInitialValueExtended(int index, float[] pose)
   {
      factorGraphExternal.setPoseInitialValueExtended(index, pose);
   }

   public void addOdometryFactorExtended(double[] odometry, int poseId)
   {
      factorGraphExternal.addOdometryFactorExtended(odometry, poseId);
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

   public void setPoseInitialValue(int index, float[] pose)
   {
      factorGraphExternal.setPoseInitialValue(index, pose);
   }

   public void setOrientedPlaneInitialValue(int landmarkId, Vector4D plane)
   {
      factorGraphExternal.setOrientedPlaneInitialValue(landmarkId, new float[] {plane.getX32(), plane.getY32(), plane.getZ32(), plane.getS32()});
   }

   public void setOrientedPlaneInitialValue(int landmarkId, float[] plane)
   {
      factorGraphExternal.setOrientedPlaneInitialValue(landmarkId, plane);
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

   public void visualSLAMTest()
   {
      factorGraphExternal.visualSLAMTest();
   }

   public void helloWorldTest()
   {
      factorGraphExternal.helloWorldTest();
   }
}
