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
   private List<ComplexNumber> rootsOfUnity = new ArrayList<>(2048);

   public RootsOfUnity(int n)
   {
      for(int i = 0; i < n; i++)
         rootsOfUnity.add(new ComplexNumber());
      unity.getRoots(rootsOfUnity, n);
   }
   
   public List<ComplexNumber> getRootsOfUnity()
   {      
      return rootsOfUnity;
   }
}
