package us.ihmc.rdx.simulation.scs2;

import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.simulation.robot.Robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Sometimes we want to restart an SCS 2 session with a button
 * and rebuild it, also possibly having changed the robot definition
 * parameters. Therefore, we want to pass in a supplier for each
 * robot definition and a consumer for when the Session gets
 * recreated, so we can pass in the instatiated Robot and setup
 * the controller, in the case that the robot needs one.
 */
public class RestartableSCS2SessionRobot
{
   private final Supplier<RobotDefinition> robotDefinitionSupplier;
   private final Consumer<Robot> setupRobotConsumer;

   public RestartableSCS2SessionRobot(Supplier<RobotDefinition> robotDefinitionSupplier, Consumer<Robot> setupRobotConsumer)
   {
      this.robotDefinitionSupplier = robotDefinitionSupplier;
      this.setupRobotConsumer = setupRobotConsumer;
   }

   public Supplier<RobotDefinition> getRobotDefinitionSupplier()
   {
      return robotDefinitionSupplier;
   }

   public Consumer<Robot> getSetupRobotConsumer()
   {
      return setupRobotConsumer;
   }
}
