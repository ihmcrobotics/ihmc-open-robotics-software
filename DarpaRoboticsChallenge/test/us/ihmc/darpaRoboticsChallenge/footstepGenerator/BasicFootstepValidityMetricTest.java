package us.ihmc.darpaRoboticsChallenge.footstepGenerator;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.configurations.DummySteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.darpaRoboticsChallenge.footstepGenerator.BasicFootstepValidityMetric;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepData;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepValidityMetric;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

/**
 * Created by agrabertilton on 2/23/15.
 */

public class BasicFootstepValidityMetricTest
{
   private static final boolean DEBUG = true;
   @DeployableTestMethod
   @Test(timeout=300000)
   public void testBasicValidityMetric(){
      SteppingParameters dummySteppingParameters = new DummySteppingParameters();
      BasicFootstepValidityMetric basicMetric = new BasicFootstepValidityMetric(dummySteppingParameters);
      testRightFootStance(basicMetric);
      testLeftFootStance(basicMetric);
   }

   private void testRightFootStance(FootstepValidityMetric metric){
      RobotSide stanceSide = RobotSide.RIGHT;
      FootstepData stanceFoot = new FootstepData(stanceSide, new Point3d(), new Quat4d());

      List<FootstepData> validFootsteps = createListOfValidFootstepsAroundZero(stanceSide);
      List<FootstepData> invalidFootsteps = createListOfInvalidFootstepsAroundZero(stanceSide);
      boolean valid;
      for (FootstepData footstep : validFootsteps){
         valid = metric.footstepValid(stanceFoot, footstep);
         if (DEBUG && !valid){
            System.out.println("Footstep should be valid " + footstep.getRobotSide() + footstep);
         }
         assertTrue(valid);
      }

      for (FootstepData footstep : invalidFootsteps){
         valid = metric.footstepValid(stanceFoot, footstep);
         if (DEBUG && valid){
            System.out.println("Footstep should be invalid " + footstep.getRobotSide() + footstep);
         }
         assertFalse(valid);
      }
   }

   private void testLeftFootStance(FootstepValidityMetric metric){
      RobotSide stanceSide = RobotSide.LEFT;
      FootstepData stanceFoot = new FootstepData(stanceSide, new Point3d(), new Quat4d());

      List<FootstepData> validFootsteps = createListOfValidFootstepsAroundZero(stanceSide);
      List<FootstepData> invalidFootsteps = createListOfInvalidFootstepsAroundZero(stanceSide);
      boolean valid;
      for (FootstepData footstep : validFootsteps){
         valid = metric.footstepValid(stanceFoot, footstep);
         if (DEBUG && !valid){
            System.out.println("Footstep should be valid " + footstep.getRobotSide() + footstep);
         }
         assertTrue(valid);
      }

      for (FootstepData footstep : invalidFootsteps){
         valid = metric.footstepValid(stanceFoot, footstep);
         if (DEBUG && valid){
            System.out.println("Footstep should be invalid " + footstep.getRobotSide() + footstep);
         }
         assertFalse(valid);
      }
   }

   private List<FootstepData> createListOfValidFootstepsAroundZero(RobotSide stanceSide){
      double sign = stanceSide == RobotSide.RIGHT? 1.0 : -1.0;
      List<FootstepData> footstepList = new ArrayList<>();
      Vector3d verticalVector = new Vector3d(0.0, 0.0, 1.0);
      Quat4d orientation = new Quat4d();

      RotationTools.computeQuaternionFromYawAndZNormal(0.0, verticalVector, orientation);
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.1, sign * 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.3, sign * 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.5, sign * 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.6, sign * 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(-0.1, sign * 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.3, sign * 0.4, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.0, sign * 0.20, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.0, sign * 0.5, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.0, sign * 0.6, 0.0), new Quat4d(orientation)));

      RotationTools.computeQuaternionFromYawAndZNormal(sign * Math.PI/8.0, verticalVector, orientation);
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.0, sign * 0.25, 0.0), new Quat4d(orientation)));

      return footstepList;
   }

   private List<FootstepData> createListOfInvalidFootstepsAroundZero(RobotSide stanceSide){
      double sign = stanceSide == RobotSide.RIGHT? 1.0 : -1.0;
      List<FootstepData> footstepList = new ArrayList<>();
      Vector3d verticalVector = new Vector3d(0.0, 0.0, 1.0);
      Quat4d orientation = new Quat4d();
      RotationTools.computeQuaternionFromYawAndZNormal(0.0, verticalVector, orientation);
      footstepList.add(new FootstepData(stanceSide, new Point3d(0.0, 0.0, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide, new Point3d(0.25, 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide, new Point3d(-0.25, 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide, new Point3d(0.25, -0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide, new Point3d(-0.25, -0.25, 0.0), new Quat4d(orientation)));

      RotationTools.computeQuaternionFromYawAndZNormal(-1 * sign * Math.PI/4.0, verticalVector, orientation);
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.0, sign * 0.25, 0.0), new Quat4d(orientation)));

      RotationTools.computeQuaternionFromYawAndZNormal(sign * Math.PI/2.0, verticalVector, orientation);
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.0, sign * 0.25, 0.0), new Quat4d(orientation)));

      RotationTools.computeQuaternionFromYawAndZNormal(0.0, verticalVector, orientation);
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(1.0, sign * 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(-1.0, sign * 0.25, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.0, sign * 1.0, 0.0), new Quat4d(orientation)));
      footstepList.add(new FootstepData(stanceSide.getOppositeSide(), new Point3d(0.0, sign * -0.25, 0.0), new Quat4d(orientation)));
      return footstepList;
   }

   private void testNonZeroOrientationStanceFoot(RobotSide stanceSide, FootstepValidityMetric metric){
      //TODO Implement this test
   }
}
