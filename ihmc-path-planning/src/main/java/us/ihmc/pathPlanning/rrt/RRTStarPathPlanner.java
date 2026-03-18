package us.ihmc.pathPlanning.rrt;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

public class RRTStarPathPlanner
{

   private final Random random = new Random();

   public List<Point3D> plan(Tuple3DReadOnly start, Tuple3DReadOnly goal, Function<Point3D, Boolean> collision)
   {
      ArrayList<Point3D> path = new ArrayList<>();

      return path;
   }



}
