package com.yobotics.simulationconstructionset;

import static org.junit.Assert.assertNotNull;

import java.util.Random;

import javax.vecmath.Vector3d;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.screwTheory.ScrewTestTools;

import com.yobotics.simulationconstructionset.RobotTools.SCSRobotFromInverseDynamicsRobotModel;

public class RobotToolsTest
{
   @Test
   public void testScsRobotFromInverseDynamicsRobotModel()
   {
      Random random = new Random();

      Vector3d[] jointAxes = {new Vector3d(1.0, 0.0, 0.0)};    // {X,Y,Z,Z,Y,X,X,Y,Z};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);

      randomFloatingChain.setRandomPositionsAndVelocities(random);

      SCSRobotFromInverseDynamicsRobotModel scsRobotFromInverseDynamicsRobotModel = new RobotTools.SCSRobotFromInverseDynamicsRobotModel("robot",
                                                                                       randomFloatingChain.getRootJoint());

      assertNotNull(scsRobotFromInverseDynamicsRobotModel);
   }

   @Ignore
   @Test
   public void testAddScsJointUsingIDJoint()
   {
   }
}
