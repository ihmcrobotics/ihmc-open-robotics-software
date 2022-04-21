package us.ihmc.avatar.ros2.networkTest;

import net.schmizz.sshj.SSHClient;
import net.schmizz.sshj.sftp.SFTPClient;
import net.schmizz.sshj.transport.verification.PromiscuousVerifier;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class SSHJTools
{
   public static void sessionWithSFTP(String address, String username, String password, Consumer<SSHJClientWithSFTP> action)
   {
      ExceptionTools.handle(() -> connectAndAuthenticateClientWithSFTP(address,
                                                                       sshClient -> ExceptionTools.handle(() -> sshClient.authPassword(username, password),
                                                                                                          DefaultExceptionHandler.MESSAGE_AND_STACKTRACE),
                                                                       action), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public static void sessionWithSFTP(String address, String username, Consumer<SSHJClientWithSFTP> action)
   {
      ExceptionTools.handle(() -> connectAndAuthenticateClientWithSFTP(address, sshClient -> authWithSSHKey(username, sshClient), action),
                            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public static void session(String address, String username, Consumer<SSHJClient> action)
   {
      ExceptionTools.handle(() -> connectAndAuthenticateClient(address, sshClient -> authWithSSHKey(username, sshClient), action),
                            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   /**
    * Replicate OpenSSH functionality where users can name private keys whatever they want.
    */
   public static void authWithSSHKey(String username, SSHClient sshClient)
   {
      Path userSSHConfigFolder = Paths.get(System.getProperty("user.home")).resolve(".ssh");

      Stream<Path> list = ExceptionTools.handle(() -> Files.list(userSSHConfigFolder), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      ArrayList<String> privateKeyFiles = new ArrayList<>();
      for (Path path : list.collect(Collectors.toList()))
      {
         if (Files.isRegularFile(path)
             && path.getFileName().toString() != "config"
             && path.getFileName().toString() != "known_hosts"
             && !path.getFileName().toString().endsWith(".pub"))
         {
            String absoluteNormalizedString = path.toAbsolutePath().normalize().toString();
            privateKeyFiles.add(absoluteNormalizedString);
         }
      }

      LogTools.info("Passing keys to authPublicKey: {}", privateKeyFiles);

      ExceptionTools.handle(() -> sshClient.authPublickey(username, privateKeyFiles.toArray(new String[0])), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public static void connectAndAuthenticateClientWithSFTP(String address,
                                                           Consumer<SSHClient> authenticate,
                                                           Consumer<SSHJClientWithSFTP> action) throws IOException
   {
      connectAndAuthenticateInternal(address, authenticate, sshClient ->
      {
         SFTPClient sftpClient = ExceptionTools.handle(sshClient::newSFTPClient, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         action.accept(new SSHJClientWithSFTP(sshClient, sftpClient));
      });
   }

   public static void connectAndAuthenticateClient(String address, Consumer<SSHClient> authenticate, Consumer<SSHJClient> action) throws IOException
   {
      connectAndAuthenticateInternal(address, authenticate, sshClient -> action.accept(new SSHJClient(sshClient)));
   }

   private static void connectAndAuthenticateInternal(String address, Consumer<SSHClient> authenticate, Consumer<SSHClient> runnable) throws IOException
   {
      SSHClient sshClient = new SSHClient();
      ExceptionTools.handle(() -> sshClient.loadKnownHosts(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      sshClient.addHostKeyVerifier(new PromiscuousVerifier()); // TODO: Try removing this again
      ExceptionTools.handle(() -> sshClient.connect(address), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      try
      {
         authenticate.accept(sshClient);
         runnable.accept(sshClient);
      }
      finally
      {
         sshClient.disconnect();
      }
   }
}
