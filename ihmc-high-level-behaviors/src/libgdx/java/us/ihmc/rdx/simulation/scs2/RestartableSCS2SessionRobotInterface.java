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
public interface RestartableSCS2SessionRobotInterface
{
   Supplier<RobotDefinition> getRobotDefinitionSupplier();

   Consumer<Robot> getSetupRobotConsumer();
}
