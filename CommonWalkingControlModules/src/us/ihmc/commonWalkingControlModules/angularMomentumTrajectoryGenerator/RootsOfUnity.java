package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.tools.ArrayTools;

/**
 * Garbage free implementation for generating the nth roots of unity 
 */
public class RootsOfUnity
{
   private static final ComplexNumber unity = new ComplexNumber(1.0, 0.0);
   private static List<ComplexNumber> rootsOfUnity = new ArrayList<>();
   public static List<ComplexNumber> getRootsOfUnity(int orderOfRoots)
   {      
      for(int i = 0; i < orderOfRoots; i++)
         rootsOfUnity.add(new ComplexNumber());
      unity.getRoots(rootsOfUnity, orderOfRoots);
      return rootsOfUnity;
   }
}
