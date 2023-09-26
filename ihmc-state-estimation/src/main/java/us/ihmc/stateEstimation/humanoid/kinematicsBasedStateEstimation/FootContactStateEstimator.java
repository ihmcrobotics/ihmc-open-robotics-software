package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public interface FootContactStateEstimator
{
   void update();

   int getNumberOfEndEffectorsTrusted();

   List<RigidBodyBasics> getListOfTrustedFeet();

   List<RigidBodyBasics> getListOfUnTrustedFeet();
}
