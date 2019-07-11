package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.PlanarRegionMessage;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.geometry.PlanarRegion;

public class PlanarRegionCommandTest
{

   @Test
   public void testClear()
   {
      PlanarRegionCommand planarRegionCommand = new PlanarRegionCommand();
      planarRegionCommand.clear();
      assertEquals(0, planarRegionCommand.getSequenceId());
      RigidBodyTransform identity = new RigidBodyTransform();
      identity.setToZero();
      assertTrue(planarRegionCommand.getTransformFromWorld().epsilonEquals(identity, 1e-9));
      assertTrue(planarRegionCommand.getTransformToWorld().epsilonEquals(identity, 1e-9));
      assertEquals(0, planarRegionCommand.getConcaveHullsVertices().size());
      assertEquals(0, planarRegionCommand.getConvexPolygons().size());
   }

   @Test
   public void testIsCommandValid()
   {
      PlanarRegionCommand planarRegionCommand = new PlanarRegionCommand();

      RigidBodyTransform transformToWorld = getRandomTransform(new Random());
      ConvexPolygon2D randomPolygon = getRandomPolygon(new Random());
      PlanarRegion randomPlanarRegion = new PlanarRegion(transformToWorld, randomPolygon);
      PlanarRegionMessage randomMessage = PlanarRegionMessageConverter.convertToPlanarRegionMessage(randomPlanarRegion);

      planarRegionCommand.setFromMessage(randomMessage);
      assertTrue(planarRegionCommand.isCommandValid());
      
      RecyclingArrayList<ConvexPolygon2D> convexPolygons = new RecyclingArrayList<ConvexPolygon2D>(10, ConvexPolygon2D.class);
      convexPolygons.add().set(randomPolygon);

      assertTrue(planarRegionCommand.getConvexPolygons().equals(convexPolygons));

      planarRegionCommand.clear();
      assertFalse(planarRegionCommand.isCommandValid());
   }

   @Test
   public void testPlanarRegionCommand()
   {
      new PlanarRegionCommand();
   }

   @Test
   public void testGetMessageClass()
   {
      PlanarRegionCommand otherPlanarRegionCommand = new PlanarRegionCommand();
      assertEquals(PlanarRegionMessage.class, otherPlanarRegionCommand.getMessageClass());
   }

   @Test
   public void testNewMessage()
   {
      PlanarRegionCommand planarRegionCommand = new PlanarRegionCommand();

      RigidBodyTransform transformToWorld = getRandomTransform(new Random());
      ConvexPolygon2D randomPolygon = getRandomPolygon(new Random());
      PlanarRegionMessage randomMessage = PlanarRegionMessageConverter.convertToPlanarRegionMessage(new PlanarRegion(transformToWorld, randomPolygon));

      planarRegionCommand.setFromMessage(randomMessage);
      assertTrue(planarRegionCommand.isCommandValid());

      ConvexPolygon2D newPolygon = getRandomPolygon(new Random());

      //The new polygon is half the dimension of the initial one
      for (int i = 0; i < randomPolygon.getNumberOfVertices(); ++i)
      {
         newPolygon.addVertex(randomPolygon.getVertex(i).getX() * 0.5, randomPolygon.getVertex(i).getY() * 0.5);
      }
      newPolygon.update();

      RecyclingArrayList<ConvexPolygon2D> convexPolygons = new RecyclingArrayList<ConvexPolygon2D>(10, ConvexPolygon2D.class);
      convexPolygons.add().set(newPolygon);
      
      randomMessage = PlanarRegionMessageConverter.convertToPlanarRegionMessage(new PlanarRegion(transformToWorld, newPolygon));

      planarRegionCommand.setFromMessage(randomMessage);
      assertTrue(planarRegionCommand.isCommandValid());

      assertTrue(planarRegionCommand.getConvexPolygons().equals(convexPolygons));

      planarRegionCommand.clear();
      assertFalse(planarRegionCommand.isCommandValid());

   }

   public RigidBodyTransform getRandomTransform(Random rand)
   {
      YawPitchRoll orientation = new YawPitchRoll(rand.nextDouble(), rand.nextDouble(), rand.nextDouble());
      Point3D translation = new Point3D(rand.nextDouble(), rand.nextDouble(), rand.nextDouble());
      return new RigidBodyTransform(orientation, translation);
   }

   public ConvexPolygon2D getRandomPolygon(Random rand)
   {
      ConvexPolygon2D randomPolygon = new ConvexPolygon2D();

      int numberOfVertices = rand.nextInt(4) + 3;

      for (int i = 0; i < numberOfVertices; ++i)
      {
         randomPolygon.addVertex(rand.nextDouble(), rand.nextDouble());
      }
      randomPolygon.update();

      return randomPolygon;
   }

   public PlanarRegionMessage getRandomPlanarRegionMessage(Random rand)
   {
      RigidBodyTransform transformToWorld = getRandomTransform(rand);
      ConvexPolygon2D randomPolygon = getRandomPolygon(rand);

      PlanarRegion randomPlanarRegion = new PlanarRegion(transformToWorld, randomPolygon);
      PlanarRegionMessage randomMessage = PlanarRegionMessageConverter.convertToPlanarRegionMessage(randomPlanarRegion);

      return randomMessage;
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionCommand.class, PlanarRegionCommandTest.class);
   }

}
