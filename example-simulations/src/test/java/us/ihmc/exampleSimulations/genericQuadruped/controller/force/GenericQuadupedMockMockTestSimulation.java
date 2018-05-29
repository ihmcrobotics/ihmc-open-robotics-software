package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedMockTestSimulation;

import java.io.IOException;

public class GenericQuadupedMockMockTestSimulation extends QuadrupedMockTestSimulation
{
   public GenericQuadupedMockMockTestSimulation() throws IOException
   {
      super();
   }

   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory(true);
   }

   public static void main(String[] args) throws IOException
   {
      new GenericQuadupedMockMockTestSimulation();
   }
}
