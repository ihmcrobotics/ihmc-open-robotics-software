package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Vector4D32;

import java.util.ArrayList;

public class MonteCarloPlannerFactory
{

    public MonteCarloPlanner create(int worldHeight, int worldWidth, int iterations, int simulations)
    {
        ArrayList<Vector4D32> obstacles = new ArrayList<>();
        Point2D agentPos = new Point2D(10, 10);
        Point2D goal = new Point2D(10, worldHeight - 10);
        int goalMargin = 5;

        Agent agent = new Agent(agentPos);
        World world = new World(obstacles, goal, goalMargin, worldHeight, worldWidth);

        return new MonteCarloPlanner(world, agent, iterations, simulations);
    }
}
